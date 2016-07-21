/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/signal.h>

#include "peripherals/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "solution.h"
#include "manage.h"
#include "simulator.h"
#include "settings.h"
#include "timing.h"
#include "base_obs.h"
#include "ephemeris.h"
#include "signal.h"
#include "iono.h"
#include "ndb.h"

extern bool disable_raim;

/** \defgroup base_obs Base station observation handling
 * \{ */

/** Mutex to control access to the base station observations. */
MUTEX_DECL(base_obs_lock);
/** Semaphore that is flagged when a new set of observations are received. */
BSEMAPHORE_DECL(base_obs_received, TRUE);
/** Most recent observations from the base station. */
obss_t base_obss;

/** Mutex to control access to the base station position state.
 * (#base_pos_ecef and #base_pos_known) */
MUTEX_DECL(base_pos_lock);
/** Is the base station position known? i.e. is #base_pos_ecef valid? */
bool base_pos_known = false;
/** Base station known position in ECEF as sent to us by the base station in
 * the BASE_POS message.  */
double base_pos_ecef[3];

/** SBP callback for when the base station sends us a message containing its
 * known location in LLH coordinates.
 * Stores the base station position in the global #base_pos_ecef variable and
 * sets #base_pos_known to `true`.
 */
static void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) sender_id;
  /*TODO: keep track of sender_id to store multiple base positions?*/
  double llh_degrees[3];
  double llh[3];
  memcpy(llh_degrees, msg, 3*sizeof(double));

  llh[0] = llh_degrees[0] * D2R;
  llh[1] = llh_degrees[1] * D2R;
  llh[2] = llh_degrees[2];

  chMtxLock(&base_pos_lock);
  wgsllh2ecef(llh, base_pos_ecef);
  base_pos_known = true;
  chMtxUnlock(&base_pos_lock);
}

/** SBP callback for when the base station sends us a message containing its
 * known location in ECEF coordinates.
 * Stores the base station position in the global #base_pos_ecef variable and
 * sets #base_pos_known to `true`.
 */
static void base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) sender_id;

  chMtxLock(&base_pos_lock);
  memcpy(base_pos_ecef, msg, 3*sizeof(double));
  base_pos_known = true;
  chMtxUnlock(&base_pos_lock);
}

/** Update the #base_obss state given a new set of obss.
 * First sorts by PRN and computes the TDCP Doppler for the observation set. If
 * #base_pos_known is false then a single point position solution is also
 * calculated. Next the `has_pos`, `pos_ecef` and `sat_dists` fields are filled
 * in. Finally the #base_obs_received semaphore is flagged to indicate that new
 * observations are available.
 *
 * \note This function is stateful as it must store the previous observation
 *       set for the TDCP Doppler.
 */
static void update_obss(obss_t *new_obss)
{
  /* Ensure observations sorted by PRN. */
  qsort(new_obss->nm, new_obss->n,
        sizeof(navigation_measurement_t), nav_meas_cmp);

  /* Lock mutex before modifying base_obss.
   * NOTE: We didn't need to lock it before reading in THIS context as this
   * is the only thread that writes to base_obss. */
  chMtxLock(&base_obs_lock);

  /* Create a set of navigation measurements to store the previous
   * observations. */
  static u8 n_old = 0;
  static gps_time_t tor_old = {.wn = 0, .tow = 0};
  static navigation_measurement_t nm_old[MAX_CHANNELS];

  /* Fill in the navigation measurements in base_obss, using TDCP method to
   * calculate the Doppler shift. */
  base_obss.n = tdcp_doppler(new_obss->n, new_obss->nm,
                             n_old, nm_old, base_obss.nm,
                             gpsdifftime(&new_obss->tor, &tor_old));

  /* Copy over sender ID. */
  base_obss.sender_id = new_obss->sender_id;

  /* Copy the current observations over to nm_old so we can difference
   * against them next time around. */
  memcpy(nm_old, new_obss->nm,
         new_obss->n * sizeof(navigation_measurement_t));
  n_old = new_obss->n;
  tor_old = new_obss->tor;

  /* Copy over the time. */
  base_obss.tor = new_obss->tor;

  u8 has_pos_old = base_obss.has_pos;
  if (base_obss.n >= 4) {
    gnss_solution soln;
    dops_t dops;

    /* check if we have fix, if yes, calculate iono and tropo correction */
    if(base_obss.has_pos) {
      double llh[3];
      wgsecef2llh(base_obss.pos_ecef, llh);
      log_debug("Base: IONO/TROPO correction");
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      if(!gps_iono_params_read(p_i_params)) {
        p_i_params = NULL;
      }
      calc_iono_tropo(base_obss.n, base_obss.nm, base_obss.pos_ecef, llh,
                      p_i_params);
    }

    /* Calculate a position solution. */
    /* disable_raim controlled by external setting (see solution.c). */
    s32 ret = calc_PVT(base_obss.n, base_obss.nm, disable_raim, &soln, &dops);

    if (ret >= 0 && soln.valid) {
      /* The position solution calculation was sucessful. Unfortunately the
       * single point position solution is very noisy so lets smooth it if we
       * have the previous position available. */
      if (has_pos_old) {
        /* TODO Implement a real filter for base position (potentially in
           observation space), so we can do away with this terrible excuse
           for smoothing. */
        base_obss.pos_ecef[0] = 0.99995 * base_obss.pos_ecef[0]
                                  + 0.00005 * soln.pos_ecef[0];
        base_obss.pos_ecef[1] = 0.99995 * base_obss.pos_ecef[1]
                                  + 0.00005 * soln.pos_ecef[1];
        base_obss.pos_ecef[2] = 0.99995 * base_obss.pos_ecef[2]
                                  + 0.00005 * soln.pos_ecef[2];
      } else {
        memcpy(base_obss.pos_ecef, soln.pos_ecef, 3 * sizeof(double));
      }
      base_obss.has_pos = 1;

      if (base_pos_known) {
       double base_distance = vector_distance(3, soln.pos_ecef, base_pos_ecef);

       if (base_distance > BASE_STATION_DISTANCE_THRESHOLD) {
         log_warn("Received base station position %f m from PVT position.",
                  base_distance);
       }
      }
    } else {
      base_obss.has_pos = 0;
      /* TODO(dsk) check for repair failure */
      /* There was an error calculating the position solution. */
      log_warn("Error calculating base station position: (%s).", pvt_err_msg[-ret-1]);
    }
  } else {
    base_obss.has_pos = 0;
  }

  /* If the base station position is known then calculate the satellite ranges.
   * This calculation will be used later by the propagation functions. */
  if (base_obss.has_pos) {
    /* Check if the base station has sent us its position explicitly via a
     * BASE_POS SBP message (as indicated by #base_pos_known).
     * No need to lock before reading here as base_pos_* is only written
     * from this thread (SBP).
     */

    for (u8 i=0; i < base_obss.n; i++) {
      base_obss.sat_dists[i] = vector_distance(3, base_obss.nm[i].sat_pos,
                                               base_obss.pos_ecef);
    }
  }

  /* Unlock base_obss mutex. */
  chMtxUnlock(&base_obs_lock);

  /* Signal that a complete base observation has been received. */
  chBSemSignal(&base_obs_received);
}

/** SBP callback for observation messages.
 * SBP observation sets are potentially split across multiple SBP messages to
 * keep the payload within the size limit.
 *
 * The header contains a count of how many total messages there are in this set
 * of observations (all referring to the same observation time) and a count of
 * which message this is in the sequence.
 *
 * This function attempts to collect a full set of observations into a single
 * `obss_t` (`base_obss_rx`). Once a full set is received then update_obss()
 * is called.
 */
static void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  /* Keep track of where in the sequence of messages we were last time around
   * so we can verify we haven't dropped a message. */
  static s16 prev_count = 0;

  static gps_time_t prev_tor = {.tow = 0.0, .wn = 0};

  /* As we receive observation messages we assemble them into a working
   * `obss_t` (`base_obss_rx`) so as not to disturb the global `base_obss`
   * state that may be in use. */
  static obss_t base_obss_rx = {.has_pos = 0};

  /* An SBP sender ID of zero means that the messages are relayed observations
   * from the console, not from the base station. We don't want to use them and
   * we don't want to create an infinite loop by forwarding them again so just
   * ignore them. */
  if (sender_id == 0) {
    return;
  }

  /* We set the sender_id */
  base_obss_rx.sender_id = sender_id;

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_OBS, len, msg, 0);

  /* GPS time of observation. */
  gps_time_t tor;
  /* Total number of messages in the observation set / sequence. */
  u8 total;
  /* The current message number in the sequence. */
  u8 count;

  /* Decode the message header to get the time and how far through the sequence
   * we are. */
  unpack_obs_header((observation_header_t*)msg, &tor, &total, &count);

  /* Check to see if the observation is aligned with our internal observations,
   * i.e. is it going to time match one of our local obs. */
  u32 obs_freq = soln_freq / obs_output_divisor;
  double epoch_count = tor.tow * obs_freq;
  double dt = fabs(epoch_count - round(epoch_count)) / obs_freq;
  if (dt > TIME_MATCH_THRESHOLD) {
    log_warn("Unaligned observation from base station ignored, "
             "tow = %.3f, dt = %.3f", tor.tow, dt);
    return;
  }

  /* Verify sequence integrity */
  if (count == 0) {
    prev_tor = tor;
    prev_count = 0;
  } else if (prev_tor.tow != tor.tow ||
             prev_tor.wn != tor.wn ||
             prev_count + 1 != count) {
    log_info("Dropped one of the observation packets! Skipping this sequence.");
    prev_count = -1;
    return;
  } else {
    prev_count = count;
  }

  /* Calculate the number of observations in this message by looking at the SBP
   * `len` field. */
  u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

  /* If this is the first packet in the sequence then reset the base_obss_rx
   * state. */
  if (count == 0) {
    base_obss_rx.n = 0;
    base_obss_rx.tor = tor;
  }

  /* Pull out the contents of the message. */
  packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
  for (u8 i=0; i<obs_in_msg; i++) {
    gnss_signal_t sid = sid_from_sbp(obs[i].sid);
    if (!sid_supported(sid))
      continue;

    /* Flag this as visible/viable to acquisition/search */
    manage_set_obs_hint(sid);

    navigation_measurement_t *nm = &base_obss_rx.nm[base_obss_rx.n];

    /* Unpack the observation into a navigation_measurement_t. */
    unpack_obs_content(&obs[i], &nm->raw_pseudorange, &nm->raw_carrier_phase,
                       &nm->snr, &nm->lock_counter, &nm->sid);

    /* Set the time */
    nm->tot = tor;
    nm->tot.tow -= nm->raw_pseudorange / GPS_C;
    normalize_gps_time(&nm->tot);

    /* Calculate satellite parameters using the ephemeris. */
    ephemeris_t ephe;
    ndb_ephemeris_read(nm->sid, &ephe);
    u8 eph_valid;
    s8 ss_ret;
    double clock_err;
    double clock_rate_err;

    eph_valid = ephemeris_valid(&ephe, &nm->tot);
    if (eph_valid) {
      ss_ret = calc_sat_state(&ephe, &nm->tot, nm->sat_pos, nm->sat_vel,
                              &clock_err, &clock_rate_err);
    }

    if (!eph_valid || (ss_ret != 0)) {
      continue;
    }

    /* Apply corrections to the raw pseudorange, carrier phase and Doppler. */
    /* TODO Make a function to apply some of these corrections.
     *      They are used in a couple places. */
    nm->pseudorange = nm->raw_pseudorange + clock_err * GPS_C;
    nm->carrier_phase = nm->raw_carrier_phase - clock_err * GPS_L1_HZ;

    /* Used in tdcp_doppler */
    nm->doppler = clock_rate_err * GPS_L1_HZ;

    /* We also apply the clock correction to the time of transmit. */
    nm->tot.tow -= clock_err;
    normalize_gps_time(&nm->tot);

    base_obss_rx.n++;
  }

  /* If we can, and all the obs have been received, update to using the new
   * obss. */
  if (count == total - 1) {
    update_obss(&base_obss_rx);
  /* Calculate packet latency. */
    if (time_quality >= TIME_COARSE) {
      gps_time_t now = get_current_time();
      float latency_ms = (float) ((now.tow - tor.tow) * 1000.0);
      log_obs_latency(latency_ms);
    }
  }
}

/** SBP callback for the old style observation messages.
 * Just logs a deprecation warning. */
static void deprecated_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) msg; (void) sender_id;
  log_warn("Received a deprecated obs msg. Verify firmware version on remote Piksi.");
}

/** Setup the base station observation handling subsystem. */
void base_obs_setup()
{
  /* Register callbacks on base station messages. */

  static sbp_msg_callbacks_node_t base_pos_llh_node;
  sbp_register_cbk(
    SBP_MSG_BASE_POS_LLH,
    &base_pos_llh_callback,
    &base_pos_llh_node
  );

  static sbp_msg_callbacks_node_t base_pos_ecef_node;
  sbp_register_cbk(
    SBP_MSG_BASE_POS_ECEF,
    &base_pos_ecef_callback,
    &base_pos_ecef_node
  );

  static sbp_msg_callbacks_node_t obs_packed_node;
  sbp_register_cbk(
    SBP_MSG_OBS,
    &obs_callback,
    &obs_packed_node
  );

  static sbp_msg_callbacks_node_t deprecated_node;
  sbp_register_cbk(
    SBP_MSG_OBS_DEP_A,
    &deprecated_callback,
    &deprecated_node
  );
  
  static sbp_msg_callbacks_node_t deprecated_node_2;
  sbp_register_cbk(
    SBP_MSG_OBS_DEP_B,
    &deprecated_callback,
    &deprecated_node_2
  );
}

/* \} */
