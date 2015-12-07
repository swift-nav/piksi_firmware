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

#include "board/leds.h"
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

extern bool disable_raim;

/** \defgroup base_obs Base station observation handling
 * \{ */

/** Mutex to control access to the base station observations. */
Mutex base_obs_lock;
/** Semaphore that is flagged when a new set of observations are received. */
BinarySemaphore base_obs_received;
/** Most recent observations from the base station. */
obss_t base_obss;

/** Mutex to control access to the base station position state.
 * (#base_pos_ecef and #base_pos_known) */
Mutex base_pos_lock;
/** Is the base station position known? i.e. is #base_pos_ecef valid? */
bool_t base_pos_known = false;
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

  double llh_degrees[3];
  double llh[3];
  memcpy(llh_degrees, msg, 3*sizeof(double));

  llh[0] = llh_degrees[0] * D2R;
  llh[1] = llh_degrees[1] * D2R;
  llh[2] = llh_degrees[2];

  chMtxLock(&base_pos_lock);
  wgsllh2ecef(llh, base_pos_ecef);
  base_pos_known = true;
  chMtxUnlock();
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
  chMtxUnlock();
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
  static navigation_measurement_t nm_old[MAX_CHANNELS];

  /* Fill in the navigation measurements in base_obss, using TDCP method to
   * calculate the Doppler shift. */
  base_obss.n = tdcp_doppler(new_obss->n, new_obss->nm,
                             n_old, nm_old, base_obss.nm);
  /* Copy over the time. */
  base_obss.t = new_obss->t;

  /* Copy the current observations over to nm_old so we can difference
   * against them next time around. */
  memcpy(nm_old, new_obss->nm,
         new_obss->n * sizeof(navigation_measurement_t));
  n_old = new_obss->n;

  /* Reset the `has_pos` flag. */
  u8 has_pos_old = base_obss.has_pos;
  base_obss.has_pos = 0;
  /* Check if the base station has sent us its position explicitly via a
   * BASE_POS SBP message (as indicated by #base_pos_known), and if so use
   * that. No need to lock before reading here as base_pos_* is only written
   * from this thread (SBP).
   */
  if (base_pos_known) {
    /* Copy the known base station position into `base_obss`. */
    memcpy(base_obss.pos_ecef, base_pos_ecef, sizeof(base_pos_ecef));
    /* Indicate that the position is valid. */
    base_obss.has_pos = 1;
  /* The base station wasn't sent to us explicitly but if we have >= 4
   * satellites we can calculate it ourselves (approximately). */
  } else if (base_obss.n >= 4) {
    gnss_solution soln;
    dops_t dops;

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
    } else {
      /* TODO(dsk) check for repair failure */
      /* There was an error calculating the position solution. */
      log_warn("Error calculating base station position: (%s).", pvt_err_msg[-ret-1]);
    }
  }
  /* If the base station position is known then calculate the satellite ranges.
   * This calculation will be used later by the propagation functions. */
  if (base_obss.has_pos) {
    for (u8 i=0; i < base_obss.n; i++) {
      double dx[3];
      vector_subtract(3, base_obss.nm[i].sat_pos, base_obss.pos_ecef, dx);
      base_obss.sat_dists[i] = vector_norm(3, dx);
    }
  }
  /* Unlock base_obss mutex. */
  chMtxUnlock();
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

  static gps_time_t prev_t = {.tow = 0.0, .wn = 0};

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

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_OBS, len, msg, 0);

  /* GPS time of observation. */
  gps_time_t t;
  /* Total number of messages in the observation set / sequence. */
  u8 total;
  /* The current message number in the sequence. */
  u8 count;

  /* Decode the message header to get the time and how far through the sequence
   * we are. */
  unpack_obs_header((observation_header_t*)msg, &t, &total, &count);

  /* Check to see if the observation is aligned with our internal observations,
   * i.e. is it going to time match one of our local obs. */
  u32 obs_freq = soln_freq / obs_output_divisor;
  double epoch_count = t.tow * obs_freq;
  double dt = fabs(epoch_count - round(epoch_count)) / obs_freq;
  if (dt > TIME_MATCH_THRESHOLD) {
    log_warn("Unaligned observation from base station ignored, "
             "tow = %.3f, dt = %.3f", t.tow, dt);
    return;
  }

  /* Calculate packet latency. */
  if (time_quality >= TIME_COARSE) {
    gps_time_t now = get_current_time();
    float latency_ms = (float) ((now.tow - t.tow) * 1000.0);
    log_obs_latency(latency_ms);
  }

  /* Verify sequence integrity */
  if (count == 0) {
    prev_t = t;
    prev_count = 0;
  } else if (prev_t.tow != t.tow ||
             prev_t.wn != t.wn ||
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
    base_obss_rx.t = t;
  }

  /* Pull out the contents of the message. */
  packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
  for (u8 i=0; i<obs_in_msg; i++) {
    /* Check the PRN is valid. e.g. simulation mode outputs test observations
     * with PRNs >200. */
    if (obs[i].sid.sat > 31) { /* TODO prn - sid; assume everything below is 0x1F masked! */
      continue;
    }

    /* Flag this as visible/viable to acquisition/search */
    manage_set_obs_hint(sid_from_sbp(obs[i].sid));

    /* Check if we have an ephemeris for this satellite, we will need this to
     * fill in satellite position etc. parameters. */
    chMtxLock(&es_mutex);
    if (ephemeris_good(&es[obs[i].sid.sat], t)) {
      /* Unpack the observation into a navigation_measurement_t. */
      unpack_obs_content(
        &obs[i],
        &base_obss_rx.nm[base_obss_rx.n].raw_pseudorange,
        &base_obss_rx.nm[base_obss_rx.n].carrier_phase,
        &base_obss_rx.nm[base_obss_rx.n].snr,
        &base_obss_rx.nm[base_obss_rx.n].lock_counter,
        &base_obss_rx.nm[base_obss_rx.n].sid
      );
      double clock_err;
      double clock_rate_err;
      /* Calculate satellite parameters using the ephemeris. */
      calc_sat_state(&es[obs[i].sid.sat], t,
                     base_obss_rx.nm[base_obss_rx.n].sat_pos,
                     base_obss_rx.nm[base_obss_rx.n].sat_vel,
                     &clock_err, &clock_rate_err);
      /* Apply corrections to the raw pseudorange. */
      /* TODO Make a function to apply some of these corrections.
       *      They are used in a couple places. */
      base_obss_rx.nm[base_obss_rx.n].pseudorange =
            base_obss_rx.nm[base_obss_rx.n].raw_pseudorange + clock_err * GPS_C;
      /* Set the time */
      base_obss_rx.nm[base_obss_rx.n].tot = t;
      base_obss_rx.n++;
    }
    chMtxUnlock();
  }

  /* If we can, and all the obs have been received, update to using the new
   * obss. */
  if (count == total - 1) {
    update_obss(&base_obss_rx);
  }
}

/** SBP callback for the old style observation messages.
 * Just logs a deprecation warning. */
static void deprecated_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) msg; (void) sender_id;
  log_error("Receiving an old deprecated observation message.");
}

/** Setup the base station observation handling subsystem. */
void base_obs_setup()
{
  /* Initialise all Mutex and Semaphore objects. */
  chMtxInit(&base_obs_lock);
  chBSemInit(&base_obs_received, TRUE);
  chMtxInit(&base_pos_lock);

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
}

/* \} */
