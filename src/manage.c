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
#include <assert.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/signal.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "acq.h"
#include "ephemeris.h"
#include "track.h"
#include "decode.h"
#include "timing.h"
#include "position.h"
#include "manage.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"
#include "peripherals/random.h"
#include "./system_monitor.h"
#include "settings.h"
#include "signal.h"

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
    ACQ_PRN_SKIP = 0,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING,
    ACQ_PRN_UNHEALTHY
  } state;                 /**< Management status of signal. */
  bool masked;             /**< Prevent acquisition. */
  u16 score[ACQ_HINT_NUM]; /**< Acquisition preference of signal. */
  float dopp_hint_low;     /**< Low bound of doppler search hint. */
  float dopp_hint_high;    /**< High bound of doppler search hint. */
  gnss_signal_t sid;       /**< Signal identifier. */
} acq_status_t;
static acq_status_t acq_status[PLATFORM_SIGNAL_COUNT];

#define SCORE_COLDSTART     100
#define SCORE_WARMSTART     200
#define SCORE_BELOWMASK     0
#define SCORE_ACQ           100
#define SCORE_TRACK         200
#define SCORE_OBS           200

#define DOPP_UNCERT_ALMANAC 4000
#define DOPP_UNCERT_EPHEM   500

static almanac_t almanac[PLATFORM_SIGNAL_COUNT];

static float elevation_mask = 0.0; /* degrees */
static bool sbas_enabled = false;

static u8 manage_track_new_acq(gnss_signal_t sid);
static void manage_acq(void);
static void manage_track(void);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)context; (void)msg;
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
  gnss_signal_t sid = sid_from_sbp(m->sid);

  char sid_str[SID_STR_LEN_MAX];
  sid_to_string(sid_str, sizeof(sid_str), sid);

  if (sid_supported(sid)) {
    acq_status_t *acq = &acq_status[sid_to_global_index(sid)];
    acq->masked = (m->mask & MASK_ACQUISITION) ? true : false;
    if (m->mask & MASK_TRACKING) {
      tracking_drop_satellite(sid);
    }
    log_info("Mask for %s = 0x%02x", sid_str, m->mask);
  } else {
    log_warn("Mask not set for invalid SID");
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

void manage_acq_setup()
{
  SETTING("acquisition", "sbas enabled", sbas_enabled, TYPE_BOOL);

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    acq_status[i].state = ACQ_PRN_ACQUIRING;
    memset(&acq_status[i].score, 0, sizeof(acq_status[i].score));
    acq_status[i].dopp_hint_low = ACQ_FULL_CF_MIN;
    acq_status[i].dopp_hint_high = ACQ_FULL_CF_MAX;
    acq_status[i].sid = sid_from_global_index(i);

    if (!sbas_enabled &&
        (sid_to_constellation(acq_status[i].sid) == CONSTELLATION_SBAS)) {
      acq_status[i].masked = true;
    }

    almanac[i].valid = 0;
  }

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
static u16 manage_warm_start(gnss_signal_t sid, const gps_time_t* t,
                             float *dopp_hint_low, float *dopp_hint_high)
{
    /* Do we have any idea where/when we are?  If not, no score. */
    /* TODO: Stricter requirement on time and position uncertainty?
       We ought to keep track of a quantitative uncertainty estimate. */
    if (time_quality < TIME_GUESS &&
        position_quality < POSITION_GUESS)
      return SCORE_COLDSTART;

    float el = 0;
    double el_d, _, dopp_hint = 0, dopp_uncertainty = DOPP_UNCERT_ALMANAC;

    /* Do we have a suitable ephemeris for this sat?  If so, use
       that in preference to the almanac. */
    const ephemeris_t *e = ephemeris_get(sid);
    if (ephemeris_valid(e, t)) {
      double sat_pos[3], sat_vel[3], el_d;
      calc_sat_state(e, t, sat_pos, sat_vel, &_, &_);
      wgsecef2azel(sat_pos, position_solution.pos_ecef, &_, &el_d);
      el = (float)(el_d) * R2D;
      if (el < elevation_mask)
        return SCORE_BELOWMASK;
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
    } else {
      const almanac_t *a = &almanac[sid_to_global_index(sid)];
      if (a->valid) {
        calc_sat_az_el_almanac(a, t->tow, t->wn-1024,
                               position_solution.pos_ecef, &_, &el_d);
        el = (float)(el_d) * R2D;
        if (el < elevation_mask)
          return SCORE_BELOWMASK;
        dopp_hint = -calc_sat_doppler_almanac(a, t->tow, t->wn,
                                              position_solution.pos_ecef);
      } else {
        return SCORE_COLDSTART; /* Couldn't determine satellite state. */
      }
    }
    /* Return the doppler hints and a score proportional to elevation */
    *dopp_hint_low = dopp_hint - dopp_uncertainty;
    *dopp_hint_high = dopp_hint + dopp_uncertainty;
    return SCORE_COLDSTART + SCORE_WARMSTART * el / 90.f;
}

static acq_status_t * choose_acq_sat(void)
{
  u32 total_score = 0;
  gps_time_t t = get_current_time();

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    if ((acq_status[i].state != ACQ_PRN_ACQUIRING) ||
        acq_status[i].masked)
      continue;

    acq_status[i].score[ACQ_HINT_WARMSTART] =
      manage_warm_start(acq_status[i].sid, &t,
                        &acq_status[i].dopp_hint_low,
                        &acq_status[i].dopp_hint_high);

    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
      total_score += acq_status[i].score[hint];
    }
  }

  if (total_score == 0) {
    log_error("Failed to pick a sat for acquisition!");
    return NULL;
  }

  u32 pick = random_int() % total_score;

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    if ((acq_status[i].state != ACQ_PRN_ACQUIRING) ||
        acq_status[i].masked)
      continue;

    u32 sat_score = 0;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      sat_score += acq_status[i].score[hint];
    if (pick < sat_score) {
      return &acq_status[i];
    } else {
      pick -= sat_score;
    }
  }

  assert(!"Error picking a sat for acquisition");
  return NULL;
}

/** Hint acqusition at satellites observed by peer.

RTK relies on a common set of measurements, have the receivers focus search
efforts on satellites both are likely to be able to see. Receiver will need
to be sufficiently close for RTK to function, and so should have a similar
view of the constellation, even if obstructed this may change if the receivers
move or the satellite arcs across the sky.

cturvey 10-Feb-2015
*/
void manage_set_obs_hint(gnss_signal_t sid)
{
  bool valid = sid_supported(sid);
  assert(valid);
  if (valid)
    acq_status[sid_to_global_index(sid)].score[ACQ_HINT_REMOTE_OBS] = SCORE_OBS;
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
static void manage_acq()
{
  /* Decide which SID to try and then start it acquiring. */
  acq_status_t *acq = choose_acq_sat();
  if (acq == NULL) {
    return;
  }

  u32 timer_count;
  float cn0, cp, cf;

  acq_set_sid(acq->sid);

  /* We have our SID chosen, now load some fresh data
   * into the acquisition ram on the Swift NAP for
   * an initial coarse acquisition.
   */
  do {
    timer_count = nap_timing_count() + 20000;
    /* acq_load could timeout if we're preempted and miss the timing strobe */
  } while (!acq_load(timer_count));

  /* Check for NaNs in dopp hints, or low > high */
  if (!(acq->dopp_hint_low <= acq->dopp_hint_high)) {
    log_error("Acq: caught bogus dopp_hints (%f, %f)",
              acq->dopp_hint_low,
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
  acq_send_result(acq->sid, cn0, cp, cf);
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

  /* Make sure a tracking channel and a decoder channel are available */
  u8 chan = manage_track_new_acq(acq->sid);
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
  tracking_channel_init(chan, acq->sid, cf, track_count, cn0,
                        TRACKING_ELEVATION_UNKNOWN);
  /* TODO: Initialize elevation from ephemeris if we know it precisely */

  /* Start the decoder channel */
  if (!decoder_channel_init(chan, acq->sid)) {
    log_error("decoder channel init failed");
  }

  acq->state = ACQ_PRN_TRACKING;
  nap_timing_strobe_wait(100);
}

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \return Index of first unused tracking channel.
 */
static u8 manage_track_new_acq(gnss_signal_t sid)
{
  /* Decide which (if any) tracking channel to put
   * a newly acquired satellite into.
   */
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if ((tracking_channel[i].state == TRACKING_DISABLED) &&
        decoder_channel_available(i, sid)) {
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

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    if (acq_status[i].state == ACQ_PRN_UNHEALTHY)
      acq_status[i].state = ACQ_PRN_ACQUIRING;
  }
}

static WORKING_AREA_CCM(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage track");
  while (TRUE) {
    chThdSleepMilliseconds(500);
    DO_EVERY(2,
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
  SETTING("solution", "elevation_mask", elevation_mask, TYPE_FLOAT);

  chThdCreateStatic(
      wa_manage_track_thread,
      sizeof(wa_manage_track_thread),
      MANAGE_TRACK_THREAD_PRIORITY,
      manage_track_thread, NULL
  );
}

static void drop_channel(u8 channel_id) {
  decoder_channel_disable(channel_id);
  tracking_channel_disable(channel_id);
  const tracking_channel_t *ch = &tracking_channel[channel_id];
  acq_status_t *acq = &acq_status[sid_to_global_index(ch->sid)];
  if (tracking_channel_running_time_ms_get(channel_id) > TRACK_REACQ_T) {
    /* FIXME other constellations/bands */
    acq->score[ACQ_HINT_PREV_TRACK] = SCORE_TRACK;
    acq->dopp_hint_low = ch->carrier_freq - ACQ_FULL_CF_STEP;
    acq->dopp_hint_high = ch->carrier_freq + ACQ_FULL_CF_STEP;
  }
  acq->state = ACQ_PRN_ACQUIRING;
}

/** Disable any tracking channel that has lost phase lock or is
    flagged unhealthy in ephem or alert flag. */
static void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {

    tracking_channel_t *ch = &tracking_channel[i];

    char buf[SID_STR_LEN_MAX];
    sid_to_string(buf, sizeof(buf), ch->sid);

    /* Skip channels that aren't in use */
    if (ch->state != TRACKING_RUNNING ||
        /* Give newly-initialized channels a chance to converge */
        tracking_channel_running_time_ms_get(i) < TRACK_INIT_T)
      continue;

    acq_status_t *acq = &acq_status[sid_to_global_index(ch->sid)];

    /* Is ephemeris or alert flag marked unhealthy?*/
    const ephemeris_t *e = ephemeris_get(ch->sid);
    /* TODO: check alert flag */
    if (e->valid && !satellite_healthy(e)) {
      log_info("%s unhealthy, dropping", buf);
      drop_channel(i);
      acq->state = ACQ_PRN_UNHEALTHY;
      continue;
    }

    /* Do we not have nav bit sync yet? */
    if (ch->bit_sync.bit_phase_ref == BITSYNC_UNSYNCED) {
      drop_channel(i);
      continue;
    }

    /* Optimistic phase lock detector "unlocked" for a while? */
    /* TODO: This isn't doing much.  Use the pessimistic detector instead? */
    if (tracking_channel_ld_opti_unlocked_ms_get(i) > TRACK_DROP_UNLOCKED_T) {
      log_info("%s PLL unlocked too long, dropping", buf);
      drop_channel(i);
      continue;
    }

    /* CN0 below threshold for a while? */
    if (tracking_channel_cn0_drop_ms_get(i) > TRACK_DROP_CN0_T) {
      log_info("%s low CN0 too long, dropping", buf);
      drop_channel(i);
      continue;
    }

    /* Is satellite below our elevation mask? */
    if (ch->elevation < elevation_mask) {
      log_info("%s below elevation mask, dropping", buf);
      drop_channel(i);
      /* Erase the tracking hint score, and any others it might have */
      memset(&acq->score, 0, sizeof(acq->score));
      continue;
    }
  }
}

s8 use_tracking_channel(u8 i)
{
  tracking_channel_t *ch = &tracking_channel[i];
  /* To use a channel's measurements in an SPP or RTK solution, we
     require the following conditions: */
  if ((ch->state == TRACKING_RUNNING)
      /* Check SNR has been above threshold for the minimum time. */
      && (tracking_channel_cn0_useable_ms_get(i) > TRACK_SNR_THRES_COUNT)
      /* Satellite elevation is above the mask. */
      && (ch->elevation >= elevation_mask)
      /* Pessimistic phase lock detector = "locked". */
      && (ch->lock_detect.outp)
      /* Some time has elapsed since the last tracking channel mode
       * change, to allow any transients to stabilize.
       * TODO: is this still necessary? */
      && (tracking_channel_last_mode_change_ms_get(i) > TRACK_STABILIZATION_T)
      /* Channel time of week has been decoded. */
      && (ch->TOW_ms != TOW_INVALID)
      /* Nav bit polarity is known, i.e. half-cycles have been resolved. */
      && (ch->bit_polarity != BIT_POLARITY_UNKNOWN)
      /* Estimated C/N0 is above some threshold */
      && tracking_channel_cn0_useable(i))
      /* TODO: Alert flag is not set */
      {
    /* Ephemeris must be valid, not stale. Satellite must be healthy.
       This also acts as a sanity check on the channel TOW.*/
    gps_time_t t = {
      /* TODO: the following makes the week number part of the
         TOW check tautological - see issue #475 */
      .wn = WN_UNKNOWN,
      .tow = 1e-3 * ch->TOW_ms
    };
    ephemeris_t *e = ephemeris_get(ch->sid);
    return ephemeris_valid(e, &t) && satellite_healthy(e);
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
