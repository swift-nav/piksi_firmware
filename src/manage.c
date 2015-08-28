/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "acq.h"
#include "track.h"
#include "timing.h"
#include "position.h"
#include "manage.h"
#include "nmea.h"
#include "sbp.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"
#include "peripherals/random.h"
#include "./system_monitor.h"

/** \defgroup manage Manage
 * Manage acquisition and tracking.
 * Manage how acquisition searches are performed, with data from almanac if
 * available. Transition from acquisition search to initializization of an
 * available tracking channel when a satellite is successfully found. Disable
 * tracking channels that have lost lock on their satellites.
 * \{ */

/** Different hints on satellite info to aid the acqusition */
enum acq_hint {
  ACQ_HINT_WARMSTART,  /**< Information from almanac or ephemeris */
  ACQ_HINT_PREV_ACQ,   /**< Previous successful acqusition. */
  ACQ_HINT_PREV_TRACK, /**< Previously tracked satellite. */
  ACQ_HINT_REMOTE_OBS, /**< Observation from reference station. */

  ACQ_HINT_NUM
};

/** Status of acquisition for a particular SID. */
typedef struct {
  enum {
    ACQ_SID_SKIP = 0,
    ACQ_SID_ACQUIRING,
    ACQ_SID_TRACKING,
    ACQ_SID_UNHEALTHY
  } state;                 /**< Management status of SID. */
  bool masked;             /**< Prevent acquisition. */
  u16 score[ACQ_HINT_NUM]; /**< Acquisition preference of SID. */
  float dopp_hint_low;     /**< Low bound of doppler search hint. */
  float dopp_hint_high;    /**< High bound of doppler search hint. */
  signal_t sid;
} acq_sid_t;
acq_sid_t l1_acq_param[GPS_L1_SATS];
acq_sid_t sbas_acq_param[WAAS_SATS];

#define SCORE_SBAS          200
#define SCORE_COLDSTART     100
#define SCORE_WARMSTART     200
#define SCORE_BELOWMASK     0
#define SCORE_ACQ           100
#define SCORE_TRACK         200
#define SCORE_OBS           200

#define DOPP_UNCERT_ALMANAC 4000
#define DOPP_UNCERT_EPHEM   500

almanac_t l1_almanac[GPS_L1_SATS];
almanac_t sbas_almanac[WAAS_SATS];

extern ephemeris_kepler_t l1_es[GPS_L1_SATS];
extern ephemeris_xyz_t sbas_es[WAAS_SATS];

ephemeris_t eph;

static float track_cn0_use_thres = 31.0; /* dBHz */
float elevation_mask = 5.0; /* degrees */

static u8 manage_track_new_acq(void);
static void manage_acq(void);
static void manage_track(void);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  int fd = -1;
  almanac_t *new_almanac = (almanac_t*)msg;

  log_info("Received alamanc for PRN %02d", new_almanac->sid.prn);
  switch(new_almanac->sid.constellation) {
    case GPS_CONSTELLATION:
      memcpy(&l1_almanac[new_almanac->sid.prn-1], new_almanac, sizeof(almanac_t));
      fd = cfs_open("l1_almanac", CFS_WRITE);
      break;
    case SBAS_CONSTELLATION:
      memcpy(&sbas_almanac[new_almanac->sid.prn-1-119], new_almanac, sizeof(almanac_t));
      fd = cfs_open("sbas_almanac", CFS_WRITE);
      break;
    default:
      log_error("Invalid constellation from received almanac!");
      return;
  }

  if (fd != -1) {
    switch(new_almanac->sid.constellation) {
      case GPS_CONSTELLATION:
        cfs_seek(fd, (new_almanac->sid.prn-1)*sizeof(almanac_t), CFS_SEEK_SET);
        break;
      case SBAS_CONSTELLATION:
        cfs_seek(fd, (new_almanac->sid.prn-1-119)*sizeof(almanac_t), CFS_SEEK_SET);
        break;
    }
    if (cfs_write(fd, new_almanac, sizeof(almanac_t)) != sizeof(almanac_t)) {
      log_error("Error writing to almanac file");
    } else {
      log_info("Saved almanac to flash");
    }
    cfs_close(fd);
  } else {
    log_error("Error opening almanac file");
  }
}

static sbp_msg_callbacks_node_t mask_sat_callback_node;
static void mask_sat_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;
  enum {
    MASK_ACQUISITION = 1,
    MASK_TRACKING = 2,
  };

  msg_mask_satellite_t *m = (msg_mask_satellite_t *)msg;

  signal_t sid = {
    .prn = m->sid.prn,
    .band = m->sid.band,
    .constellation = m->sid.constellation
  };
  u16 prn = sid.prn;

  switch(m->sid.constellation){
    case GPS_CONSTELLATION:
      log_info("Mask for PRN %02d = 0x%02x", prn+1, m->mask);
      if (m->mask & MASK_ACQUISITION) {
        l1_acq_param[prn].masked = true;
      } else {
        l1_acq_param[prn].masked = false;
      }
      if (m->mask & MASK_TRACKING) {
        tracking_drop_satellite(prn);
      }
      break;
    case SBAS_CONSTELLATION:
      log_info("Mask for PRN %02d = 0x%02x", prn+1, m->mask);
      if (m->mask & MASK_ACQUISITION) {
        sbas_acq_param[sbas_sid_to_index(sid)].masked = true;
      } else {
        sbas_acq_param[sbas_sid_to_index(sid)].masked = false;
      }
      if (m->mask & MASK_TRACKING) {
        tracking_drop_satellite(prn); /*TODO: look here!!!!*/
      }
      break;
  }
}

static WORKING_AREA_CCM(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static msg_t manage_acq_thread(void *arg)
{
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */
  (void)arg;
  chRegSetThreadName("manage acq");
  while (TRUE) {
    manage_acq();
    watchdog_notify(WD_NOTIFY_ACQ_MGMT);
  }

  return 0;
}

static void l1_setup_acq()
{
  for (u8 prn=0; prn<GPS_L1_SATS; prn++) {
    l1_acq_param[prn].state = ACQ_SID_ACQUIRING;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      l1_acq_param[prn].score[hint] = 0;
    l1_acq_param[prn].dopp_hint_low = ACQ_FULL_CF_MIN;
    l1_acq_param[prn].dopp_hint_high = ACQ_FULL_CF_MAX;
    l1_acq_param[prn].sid.prn = prn;
    l1_acq_param[prn].sid.constellation = GPS_CONSTELLATION;
    l1_acq_param[prn].sid.band = L1_BAND;
  }
}

static void l1_setup_almanac()
{
  int fd = cfs_open("l1_almanac", CFS_READ);
  if (fd != -1) {
    cfs_read(fd, l1_almanac, GPS_L1_SATS*sizeof(almanac_t));
    log_info("Loaded GPS/L1 almanac from flash.");
    cfs_close(fd);
  } else {
    log_info("No GPS/L1 almanac file present in flash, create an empty one.");
    cfs_coffee_reserve("l1_almanac", GPS_L1_SATS*sizeof(almanac_t));
    cfs_coffee_configure_log("l1_almanac", 256, sizeof(almanac_t));

    for (u8 prn=0; prn<GPS_L1_SATS; prn++) {
      l1_almanac[prn].valid = 0;
    }
  }
}

static void sbas_setup_acq()
{
  for (u8 prn=0; prn<WAAS_SATS; prn++) {
    sbas_acq_param[prn].state = ACQ_SID_ACQUIRING;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      sbas_acq_param[prn].score[hint] = 0;
    sbas_acq_param[prn].dopp_hint_low = ACQ_FULL_CF_MIN;
    sbas_acq_param[prn].dopp_hint_high = ACQ_FULL_CF_MAX;
    sbas_acq_param[prn].sid = sbas_index_to_sid(prn);
  }
}

static void sbas_setup_almanac()
{
  int fd = cfs_open("sbas_almanac", CFS_READ);
  if (fd != -1) {
    cfs_read(fd, sbas_almanac, WAAS_SATS*sizeof(almanac_t));
    log_info("Loaded SBAS almanac from flash.");
    cfs_close(fd);
  } else {
    log_info("No SBAS almanac file present in flash, create an empty one.");
    cfs_coffee_reserve("sbas_almanac", WAAS_SATS*sizeof(almanac_t));
    cfs_coffee_configure_log("sbas_almanac", 256, sizeof(almanac_t));

    for (u8 prn=0; prn<WAAS_SATS; prn++) {
      sbas_almanac[prn].valid = 0;
    }
  }
}

static void ephemeris_setup()
{
  eph.ephemeris_kep = l1_es;
  eph.ephemeris_xyz = sbas_es;
}

void manage_acq_setup()
{
  l1_setup_acq();
  l1_setup_almanac();

  sbas_setup_acq();
  sbas_setup_almanac();

  ephemeris_setup();

  sbp_register_cbk(
    SBP_MSG_ALMANAC,
    &almanac_callback,
    &almanac_callback_node
  );

  sbp_register_cbk(
    SBP_MSG_MASK_SATELLITE,
    &mask_sat_callback,
    &mask_sat_callback_node
  );

  chThdCreateStatic(
      wa_manage_acq_thread,
      sizeof(wa_manage_acq_thread),
      MANAGE_ACQ_THREAD_PRIORITY,
      manage_acq_thread, NULL
  );
}


/** Using available almanac and ephemeris information, determine
 * whether a satellite is in view and the range of doppler frequencies
 * in which we expect to find it.
 *
 * \param prn 0-indexed PRN
 * \param t Time at which to evaluate ephemeris and almanac (typically system's
 *  estimate of current time)
 * \param dopp_hint_low, dopp_hint_high Pointers to store doppler search range
 *  from ephemeris or almanac, if available and elevation > mask
 * \return Score (higher is better)
 */
static u16 manage_warm_start(signal_t sid, gps_time_t t,
                             float *dopp_hint_low, float *dopp_hint_high)
{
    /* Do we have any idea where/when we are?  If not, no score. */
    /* TODO: Stricter requirement on time and position uncertainty?
       We ought to keep track of a quantitative uncertainty estimate. */
    if (time_quality < TIME_GUESS &&
        position_quality < POSITION_GUESS) {
      return SCORE_COLDSTART;
    }

    float el = 0;
    double el_d, _, dopp_hint = 0, dopp_uncertainty = DOPP_UNCERT_ALMANAC;

    if (sid.constellation == SBAS_CONSTELLATION) {
       *dopp_hint_low = dopp_hint - dopp_uncertainty;
       *dopp_hint_high = dopp_hint + dopp_uncertainty;
      return SCORE_SBAS;
    }
        /* Do we have a suitable ephemeris for this sat?  If so, use
       that in preference to the almanac. */

    almanac_t *alm = &l1_almanac[sid.prn];

    ephemeris_t e;
    if (sid.constellation == GPS_CONSTELLATION) {
      e.ephemeris_kep = &eph.ephemeris_kep[sid.prn];
      e.ephemeris_xyz = NULL;
    } else {
      e.ephemeris_xyz = &eph.ephemeris_xyz[sbas_sid_to_index(sid)];
      e.ephemeris_kep = NULL;
      alm = &sbas_almanac[sbas_sid_to_index(sid)];
    }
    //TODO for calc_sat_state
    ephemeris_kepler_t kep = eph.ephemeris_kep[sid.prn];

    if (ephemeris_good(&e, sid, t)) {
      double sat_pos[3], sat_vel[3], el_d;
      calc_sat_state(&kep, t, sat_pos, sat_vel, &_, &_);
      wgsecef2azel(sat_pos, position_solution.pos_ecef, &_, &el_d);
      el = (float)(el_d) * R2D;
      if (el < elevation_mask) {
        return SCORE_BELOWMASK;
      }
      vector_subtract(3, sat_pos, position_solution.pos_ecef, sat_pos);
      vector_normalize(3, sat_pos);
      /* sat_pos now holds unit vector from us to satellite */
      vector_subtract(3, sat_vel, position_solution.vel_ecef, sat_vel);
      /* sat_vel now holds velocity of sat relative to us */
      dopp_hint = -GPS_L1_HZ * (vector_dot(3, sat_pos, sat_vel) / GPS_C
                                + position_solution.clock_bias);
      /* TODO: Check sign of receiver frequency offset correction */
      if (time_quality >= TIME_FINE)
        dopp_uncertainty = DOPP_UNCERT_EPHEM;
    } else if (alm->valid) {
      calc_sat_az_el_almanac(alm, t.tow, t.wn-1024,
                             position_solution.pos_ecef, &_, &el_d);
      el = (float)(el_d) * R2D;
      if (el < elevation_mask)
        return SCORE_BELOWMASK;
      dopp_hint = -calc_sat_doppler_almanac(alm, t.tow, t.wn,
                                            position_solution.pos_ecef);
    } else {
      return SCORE_COLDSTART; /* Couldn't determine satellite state. */
    }
    /* Return the doppler hints and a score proportional to elevation */
    *dopp_hint_low = dopp_hint - dopp_uncertainty;
    *dopp_hint_high = dopp_hint + dopp_uncertainty;
    return SCORE_COLDSTART + SCORE_WARMSTART * el / 90.f;
}

static signal_t choose_sid(void)
{
  u32 total_score = 0;
  gps_time_t t = get_current_time();

  for (u8 prn=0; prn<GPS_L1_SATS + WAAS_SATS; prn++) {
    if (prn < GPS_L1_SATS) {
      if ((l1_acq_param[prn].state != ACQ_SID_ACQUIRING) ||
           l1_acq_param[prn].masked)
        continue;

      l1_acq_param[prn].score[ACQ_HINT_WARMSTART] =
        manage_warm_start(l1_acq_param[prn].sid, t,
                          &l1_acq_param[prn].dopp_hint_low,
                          &l1_acq_param[prn].dopp_hint_high);
      for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
        total_score += l1_acq_param[prn].score[hint];
      }
    } else {
      u8 local_prn = prn - GPS_L1_SATS;
      if ((sbas_acq_param[local_prn].state != ACQ_SID_ACQUIRING) ||
           sbas_acq_param[local_prn].masked)
        continue;

      sbas_acq_param[local_prn].score[ACQ_HINT_WARMSTART] =
        manage_warm_start(sbas_acq_param[local_prn].sid, t,
                          &sbas_acq_param[local_prn].dopp_hint_low,
                          &sbas_acq_param[local_prn].dopp_hint_high);
      for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
        total_score += sbas_acq_param[local_prn].score[hint];
      }
    }
  }

  u32 pick = random_int() % total_score;

  for (u8 prn=0; prn<GPS_L1_SATS + WAAS_SATS; prn++) {
    if (prn < GPS_L1_SATS) {
        if ((l1_acq_param[prn].state != ACQ_SID_ACQUIRING) ||
           l1_acq_param[prn].masked)
        continue;

      u32 sat_score = 0;
      for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
        sat_score += l1_acq_param[prn].score[hint];
      if (pick < sat_score) {
        return l1_acq_param[prn].sid;
      } else {
        pick -= sat_score;
      }
    } else {
      u8 local_prn = prn - GPS_L1_SATS;
      if ((sbas_acq_param[local_prn].state != ACQ_SID_ACQUIRING) ||
           sbas_acq_param[local_prn].masked)
        continue;

      u32 sat_score = 0;
      for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
        sat_score += sbas_acq_param[local_prn].score[hint];
      if (pick < sat_score) {
        return sbas_acq_param[local_prn].sid;
      } else {
        pick -= sat_score;
      }
    }
  }

  log_error("Failed to pick a sat for acquisition!");

  signal_t fail = {
    .prn = -1,
    .band = L1_BAND,
    .constellation = GPS_CONSTELLATION
  };

  return fail;
}

/** Hint acqusition at satellites observed by peer.

RTK relies on a common set of measurements, have the receivers focus search
efforts on satellites both are likely to be able to see. Receiver will need
to be sufficiently close for RTK to function, and so should have a similar
view of the constellation, even if obstructed this may change if the receivers
move or the satellite arcs across the sky.

cturvey 10-Feb-2015
*/
void manage_set_obs_hint(u16 prn)
{
  if (prn >= 32) /* check range */
    return;

  l1_acq_param[prn].score[ACQ_HINT_REMOTE_OBS] = SCORE_OBS;
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
static void manage_acq()
{
  /* Decide which PRN to try and then start it acquiring. */
  signal_t sid = choose_sid();
  if (sid.prn == (u16)-1)
    return;

  u32 timer_count;
  float cn0, cp, cf;

  acq_set_sid(sid);

  /* We have our PRN chosen, now load some fresh data
   * into the acquisition ram on the Swift NAP for
   * an initial coarse acquisition.
   */
  do {
    timer_count = nap_timing_count() + 20000;
    /* acq_load could timeout if we're preempted and miss the timing strobe */
  } while (!acq_load(timer_count));

  acq_sid_t *acq = &l1_acq_param[sid.prn];
  if (sid.constellation == SBAS_CONSTELLATION)
    acq = &sbas_acq_param[sbas_sid_to_index(sid)];

  /* Check for NaNs in dopp hints, or low > high */
  if (!(acq->dopp_hint_low <= acq->dopp_hint_high)) {
    log_error("Acq: caught bogus dopp_hints (%f, %f)", acq->dopp_hint_low,
              acq->dopp_hint_high);
    acq->dopp_hint_high = ACQ_FULL_CF_MAX;
    acq->dopp_hint_low = ACQ_FULL_CF_MIN;
  }
  acq_search(acq->dopp_hint_low, acq->dopp_hint_high, ACQ_FULL_CF_STEP);

  /* Done with the coarse acquisition, check if we have found a
   * satellite, if so save the results and start the loading
   * for the fine acquisition. If not, start again choosing a
   * different PRN.
   */
  acq_get_results(&cp, &cf, &cn0);
  /* Send result of an acquisition to the host. */
  acq_send_result(sid, cn0, cp, cf);
  if (cn0 < ACQ_THRESHOLD) {
    /* Didn't find the satellite :( */
    /* Double the size of the doppler search space for next time. */
    float dilute = (acq->dopp_hint_high - acq->dopp_hint_low) / 2;
    acq->dopp_hint_high = MIN(acq->dopp_hint_high + dilute, ACQ_FULL_CF_MAX);
    acq->dopp_hint_low = MAX(acq->dopp_hint_low - dilute, ACQ_FULL_CF_MIN);
    /* Decay hint scores */
    for (u8 i = 0; i < ACQ_HINT_NUM; i++)
      acq->score[i] = (acq->score[i] * 3) / 4;
    /* Reset hint score for acquisition. */
    acq->score[ACQ_HINT_PREV_ACQ] = 0;
    return;
  }

  u8 chan = manage_track_new_acq();
  if (chan == MANAGE_NO_CHANNELS_FREE) {
    /* No channels are free to accept our new satellite :( */
    /* TODO: Perhaps we can try to warm start this one
     * later using another fine acq.
     */
    if (cn0 > ACQ_RETRY_THRESHOLD) {
      acq->score[ACQ_HINT_PREV_ACQ] = SCORE_ACQ + (cn0 - ACQ_THRESHOLD);
      acq->dopp_hint_low = cf - ACQ_FULL_CF_STEP;
      acq->dopp_hint_high = cf + ACQ_FULL_CF_STEP;
    }
    return;
  }
  /* Transition to tracking. */
  u32 track_count = nap_timing_count() + 20000;
  cp = propagate_code_phase(cp, cf, track_count - timer_count);

  // Contrive for the timing strobe to occur at or close to a PRN edge (code phase = 0)
  track_count += 16*(1023.0-cp)*(1.0 + cf / GPS_L1_HZ);

  /* Start the tracking channel */
  tracking_channel_init(chan, sid, cf, track_count, cn0,
                        TRACKING_ELEVATION_UNKNOWN);
  /* TODO: Initialize elevation from ephemeris if we know it precisely */

  acq->state = ACQ_SID_TRACKING;
  nap_timing_strobe_wait(100);
}

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \return Index of first unused tracking channel.
 */
static u8 manage_track_new_acq(void)
{
  /* Decide which (if any) tracking channel to put
   * a newly acquired satellite into.
   */
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (tracking_channel[i].state == TRACKING_DISABLED) {
      return i;
    }
  }

  return MANAGE_NO_CHANNELS_FREE;
}

/** Clear unhealthy flags after some time, so we eventually retry
    those sats in case they recover from their sickness.  Call this
    function regularly, and once per day it will reset the flags. */
static void check_clear_unhealthy(void)
{
  static systime_t ticks;
  if (chTimeElapsedSince(ticks) < S2ST(24*60*60))
    return;

  ticks = chTimeNow();

  for (u8 prn=0; prn<GPS_L1_SATS; prn++) {
    if (l1_acq_param[prn].state == ACQ_SID_UNHEALTHY)
      l1_acq_param[prn].state = ACQ_SID_ACQUIRING;
  }

  for (u8 prn=0; prn<WAAS_SATS; prn++) {
    if (sbas_acq_param[prn].state == ACQ_SID_UNHEALTHY)
      sbas_acq_param[prn].state = ACQ_SID_ACQUIRING;
  }
}

static WORKING_AREA_CCM(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage track");
  while (TRUE) {
    chThdSleepMilliseconds(200);
    DO_EVERY(5,
      check_clear_unhealthy();
      manage_track();
      nmea_gpgsa(tracking_channel, 0);
      watchdog_notify(WD_NOTIFY_TRACKING_MGMT);
    );
    tracking_send_state();
  }

  return 0;
}

void manage_track_setup()
{
  initialize_lock_counters();

  SETTING("track", "cn0_use", track_cn0_use_thres, TYPE_FLOAT);
  SETTING("solution", "elevation_mask", elevation_mask, TYPE_FLOAT);

  chThdCreateStatic(
      wa_manage_track_thread,
      sizeof(wa_manage_track_thread),
      MANAGE_TRACK_THREAD_PRIORITY,
      manage_track_thread, NULL
  );
}

static void drop_channel(u8 channel_id) {
  tracking_channel_disable(channel_id);
  const tracking_channel_t *ch = &tracking_channel[channel_id];
  acq_sid_t *acq = &l1_acq_param[ch->sid.prn];

  if (ch->sid.constellation == SBAS_CONSTELLATION)
    acq = &sbas_acq_param[sbas_sid_to_index(ch->sid)];

  if (ch->update_count > TRACK_REACQ_T) {
    acq->score[ACQ_HINT_PREV_TRACK] = SCORE_TRACK;
    acq->dopp_hint_low = ch->carrier_freq - ACQ_FULL_CF_STEP;
    acq->dopp_hint_high = ch->carrier_freq + ACQ_FULL_CF_STEP;
  }
  acq->state = ACQ_SID_ACQUIRING;
}

/** Disable any tracking channel that has lost phase lock or is
    flagged unhealthy in ephem. */
static void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {

    tracking_channel_t *ch = &tracking_channel[i];
    signal_t sid = ch->sid;
    acq_sid_t *acq = NULL;
    ephemeris_t e;

    if (sid.constellation == GPS_CONSTELLATION) {
      acq = &l1_acq_param[sid.prn];
      e.ephemeris_kep = &eph.ephemeris_kep[sid.prn];
      e.ephemeris_xyz = NULL;
    } else {
      acq = &sbas_acq_param[sbas_sid_to_index(sid)];
      e.ephemeris_xyz = &eph.ephemeris_xyz[sbas_sid_to_index(sid)];
      e.ephemeris_kep = NULL;
    }

    /* Skip channels that aren't in use */
    if (ch->state != TRACKING_RUNNING ||
        /* Give newly-initialized channels a chance to converge */
        ch->update_count < TRACK_INIT_T)
      continue;

    /* Is ephemeris marked unhealthy? */
    u8 status = 0;
    if (sid.constellation == GPS_CONSTELLATION) {
      if (e.ephemeris_kep[0].valid && !e.ephemeris_kep[0].healthy)
        status = 1;
    } else {
      if (e.ephemeris_xyz[0].valid && !e.ephemeris_xyz[0].healthy)
        status = 1;
    }

    if (status == 1) {
      log_info("PRN%d unhealthy, dropping", ch->sid.prn+1);
      drop_channel(i);
      acq->state = ACQ_SID_UNHEALTHY;
      continue;
    }

    /* Do we not have nav bit sync yet? */
    s8 *b_phase_ref;
    if (sid.constellation == GPS_CONSTELLATION) {
        b_phase_ref = &ch->nav_msg.l1_nav_msg->bit_phase_ref;
      } else {
        b_phase_ref = &ch->nav_msg.sbas_nav_msg->bit_phase_ref;
      }
    if (*b_phase_ref == BITSYNC_UNSYNCED) {
      drop_channel(i);
      continue;
    }

    u32 uc = ch->update_count;

    /* Optimistic phase lock detector "unlocked" for a while? */
    /* TODO: This isn't doing much.  Use the pessimistic detector instead? */
    if ((int)(uc - ch->ld_opti_locked_count) > TRACK_DROP_UNLOCKED_T) {
      log_info("PRN%d PLL unlocked too long, dropping", ch->sid.prn+1);
      drop_channel(i);
      continue;
    }

    /* CN0 below threshold for a while? */
    if ((int)(uc - ch->cn0_above_drop_thres_count) > TRACK_DROP_CN0_T) {
      log_info("PRN%d low CN0 too long, dropping", ch->sid.prn+1);
      drop_channel(i);
      continue;
    }

    /* Is satellite below our elevation mask? */
    if (ch->elevation < elevation_mask) {
      log_info("PRN%d below elevation mask, dropping", ch->sid.prn+1);
      drop_channel(i);
      /* Erase the tracking hint score, and any others it might have */
      memset(acq->score, 0, sizeof(acq->score));
      continue;
    }
  }
}

s8 use_tracking_channel(u8 i)
{
  tracking_channel_t *ch = &tracking_channel[i];
  signal_t sid = ch->sid;
  ephemeris_t e;
  s8 *b_polarity;

  if (sid.constellation == GPS_CONSTELLATION)
    b_polarity = &ch->nav_msg.l1_nav_msg->bit_polarity;
  else
    b_polarity = &ch->nav_msg.sbas_nav_msg->bit_polarity;

  if (sid.constellation == GPS_CONSTELLATION) {
    e.ephemeris_kep = &eph.ephemeris_kep[sid.prn];
    e.ephemeris_xyz = NULL;
  } else {
    e.ephemeris_xyz = &eph.ephemeris_xyz[sbas_sid_to_index(sid)];
    e.ephemeris_kep = NULL;
  }

  /* To use a channel's measurements in an SPP or RTK solution, we
     require the following conditions: */
  if ((ch->state == TRACKING_RUNNING)
      /* Satellite elevation is above the mask. */
      && (ch->elevation >= elevation_mask)
      /* Pessimistic phase lock detector = "locked". */
      && (ch->lock_detect.outp)
      /* Some time has elapsed since the last tracking channel mode
       * change, to allow any transients to stabilize.
       * TODO: is this still necessary? */
      && ((int)(ch->update_count - ch->mode_change_count) > TRACK_STABILIZATION_T)
      /* Channel time of week has been decoded. */
      && (ch->TOW_ms != TOW_INVALID)
      /* Nav bit polarity is known, i.e. half-cycles have been resolved. */
      && (*b_polarity != BIT_POLARITY_UNKNOWN)
      /* Estimated C/N0 is above some threshold */
      && (ch->cn0 > track_cn0_use_thres))
      {
    /* Ephemeris must be valid and not stale.
       This also acts as a sanity check on the channel TOW.*/
    gps_time_t t = {
      /* TODO: the following makes the week number part of the
         TOW check tautological - see issue #475 */
      .wn = WN_UNKNOWN,
      .tow = 1e-3 * ch->TOW_ms
    };
    return ephemeris_good(&e, sid, t);
  } else return 0;
}

u8 tracking_channels_ready()
{
  u8 n_ready = 0;
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (use_tracking_channel(i)) {
      n_ready++;
    }
  }
  return n_ready;
}

/** \} */
