/*
 * Copyright (C) 2011-2014,2016 Swift Navigation Inc.
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
#include <stdlib.h>
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
#include <libswiftnav/constants.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "board/acq.h"
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

static bool track_mask[PLATFORM_SIGNAL_COUNT];

#define SCORE_COLDSTART     100
#define SCORE_WARMSTART     200
#define SCORE_BELOWMASK     0
#define SCORE_ACQ           100
#define SCORE_TRACK         200
#define SCORE_OBS           200

#define DOPP_UNCERT_ALMANAC 4000
#define DOPP_UNCERT_EPHEM   500

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define TRACKING_STARTUP_FIFO_SIZE 8    /* Must be a power of 2 */

#define TRACKING_STARTUP_FIFO_INDEX_MASK ((TRACKING_STARTUP_FIFO_SIZE) - 1)
#define TRACKING_STARTUP_FIFO_INDEX_DIFF(write_index, read_index) \
          ((tracking_startup_fifo_index_t)((write_index) - (read_index)))
#define TRACKING_STARTUP_FIFO_LENGTH(p_fifo) \
          (TRACKING_STARTUP_FIFO_INDEX_DIFF((p_fifo)->write_index, \
                                            (p_fifo)->read_index))

typedef u8 tracking_startup_fifo_index_t;

typedef struct {
  tracking_startup_fifo_index_t read_index;
  tracking_startup_fifo_index_t write_index;
  tracking_startup_params_t elements[TRACKING_STARTUP_FIFO_SIZE];
} tracking_startup_fifo_t;

static tracking_startup_fifo_t tracking_startup_fifo;

static MUTEX_DECL(tracking_startup_mutex);

static almanac_t almanac[PLATFORM_SIGNAL_COUNT];

static float elevation_mask = 0.0; /* degrees */
static bool sbas_enabled = false;

static void acq_result_send(gnss_signal_t sid, float snr, float cp, float cf);

static u8 manage_track_new_acq(gnss_signal_t sid);
static void manage_acq(void);
static void manage_track(void);

static void manage_tracking_startup(void);
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo);
static bool tracking_startup_fifo_write(tracking_startup_fifo_t *fifo,
                                        const tracking_startup_params_t *
                                        element);
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element);

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

  if (sid_supported(sid)) {
    u16 global_index = sid_to_global_index(sid);
    acq_status_t *acq = &acq_status[global_index];
    acq->masked = (m->mask & MASK_ACQUISITION) ? true : false;
    track_mask[global_index] = (m->mask & MASK_TRACKING) ? true : false;
    log_info_sid(sid, "Mask = 0x%02x", m->mask);
  } else {
    log_warn("Mask not set for invalid SID");
  }
}

static WORKING_AREA_BCKP(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static void manage_acq_thread(void *arg)
{
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */
  (void)arg;
  chRegSetThreadName("manage acq");
  while (TRUE) {
    manage_acq();
    manage_tracking_startup();
    watchdog_notify(WD_NOTIFY_ACQ_MGMT);
  }
}

void manage_acq_setup()
{
  SETTING("acquisition", "sbas enabled", sbas_enabled, TYPE_BOOL);

  tracking_startup_fifo_init(&tracking_startup_fifo);

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    acq_status[i].state = ACQ_PRN_ACQUIRING;
    acq_status[i].masked = false;
    memset(&acq_status[i].score, 0, sizeof(acq_status[i].score));
    acq_status[i].dopp_hint_low = ACQ_FULL_CF_MIN;
    acq_status[i].dopp_hint_high = ACQ_FULL_CF_MAX;
    acq_status[i].sid = sid_from_global_index(i);

    track_mask[i] = false;
    almanac[i].valid = 0;

    if (!sbas_enabled &&
        (sid_to_constellation(acq_status[i].sid) == CONSTELLATION_SBAS)) {
      acq_status[i].masked = true;
      track_mask[i] = true;
    }
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
    bool ready = false;
    /* Do we have a suitable ephemeris for this sat?  If so, use
       that in preference to the almanac. */
    const ephemeris_t *e = ephemeris_get(sid);
    if (ephemeris_valid(e, t)) {
      double sat_pos[3], sat_vel[3], el_d;
      if (calc_sat_state(e, t, sat_pos, sat_vel, &_, &_) == 0) {
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
        ready = true;
      }
    }

    if(!ready) {
      const almanac_t *a = &almanac[sid_to_global_index(sid)];
      if (a->valid &&
          calc_sat_az_el_almanac(a, t, position_solution.pos_ecef,
                                 &_, &el_d) == 0) {
          el = (float)(el_d) * R2D;
          if (el < elevation_mask)
            return SCORE_BELOWMASK;
          if (calc_sat_doppler_almanac(a, t, position_solution.pos_ecef,
                                       &dopp_hint) != 0) {
            return SCORE_COLDSTART;
          }
          dopp_hint = -dopp_hint;
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
    if (!code_requires_direct_acq(acq_status[i].sid.code)) {
      continue;
    }

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

  u32 pick = rand() % total_score;

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    if (!code_requires_direct_acq(acq_status[i].sid.code)) {
      continue;
    }

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

  /* Check for NaNs in dopp hints, or low > high */
  if (!(acq->dopp_hint_low <= acq->dopp_hint_high)) {
    log_error("Acq: caught bogus dopp_hints (%f, %f)",
              acq->dopp_hint_low,
              acq->dopp_hint_high);
    acq->dopp_hint_high = ACQ_FULL_CF_MAX;
    acq->dopp_hint_low = ACQ_FULL_CF_MIN;
  }

  acq_result_t acq_result;
  if (acq_search(acq->sid, acq->dopp_hint_low, acq->dopp_hint_high,
                 ACQ_FULL_CF_STEP, &acq_result)) {

    /* Send result of an acquisition to the host. */
    acq_result_send(acq->sid, acq_result.cn0, acq_result.cp, acq_result.cf);

    if (acq_result.cn0 < ACQ_THRESHOLD) {
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

    tracking_startup_params_t tracking_startup_params = {
      .sid = acq->sid,
      .sample_count = acq_result.sample_count,
      .carrier_freq = acq_result.cf,
      .code_phase = acq_result.cp,
      .chips_to_correlate = GPS_L1CA_CHIPS_NUM,
      .cn0_init = acq_result.cn0,
      .elevation = TRACKING_ELEVATION_UNKNOWN
    };

    tracking_startup_request(&tracking_startup_params);
  }
}

/** Send results of an acquisition to the host.
 *
 * \param sid SID of the acquisition
 * \param snr Signal to noise ratio of best point from acquisition.
 * \param cp  Code phase of best point.
 * \param cf  Carrier frequency of best point.
 */
static void acq_result_send(gnss_signal_t sid, float snr, float cp, float cf)
{
  msg_acq_result_t acq_result_msg;

  acq_result_msg.sid = sid_to_sbp(sid);
  acq_result_msg.snr = snr;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = cf;

  sbp_send_msg(SBP_MSG_ACQ_RESULT,
               sizeof(msg_acq_result_t),
               (u8 *)&acq_result_msg);
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
    if (tracker_channel_available(i, sid) &&
        /** \todo: the (sid.code == 1) part is to be removed once L2C
                   data decoding channel support is added */
        (decoder_channel_available(i, sid) || (sid.code == 1))) {
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
  if (chVTTimeElapsedSinceX(ticks) < S2ST(24*60*60))
    return;

  ticks = chVTGetSystemTime();

  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    if (acq_status[i].state == ACQ_PRN_UNHEALTHY)
      acq_status[i].state = ACQ_PRN_ACQUIRING;
  }
}

static WORKING_AREA_BCKP(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static void manage_track_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage track");
  while (TRUE) {
    chThdSleepMilliseconds(500);
    DO_EVERY(2,
      check_clear_unhealthy();
      manage_track();
      watchdog_notify(WD_NOTIFY_TRACKING_MGMT);
    );
    tracking_send_state();
  }
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
  /* Read the required parameters from the tracking channel first to ensure
   * that the tracking channel is not restarted in the mean time.
   */
  gnss_signal_t sid = tracking_channel_sid_get(channel_id);
  double carrier_freq = tracking_channel_carrier_freq_get(channel_id);
  acq_status_t *acq = &acq_status[sid_to_global_index(sid)];
  if (tracking_channel_running_time_ms_get(channel_id) > TRACK_REACQ_T) {
    /* FIXME other constellations/bands */
    acq->score[ACQ_HINT_PREV_TRACK] = SCORE_TRACK;
    acq->dopp_hint_low = carrier_freq - ACQ_FULL_CF_STEP;
    acq->dopp_hint_high = carrier_freq + ACQ_FULL_CF_STEP;
  }
  acq->state = ACQ_PRN_ACQUIRING;

  /* Finally disable the decoder and tracking channels */
  decoder_channel_disable(channel_id);
  tracker_channel_disable(channel_id);
}

/** Disable any tracking channel that has lost phase lock or is
    flagged unhealthy in ephem or alert flag. */
static void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {

    /* Skip channels that aren't in use */
    if (!tracking_channel_running(i)) {
      continue;
    }

    gnss_signal_t sid = tracking_channel_sid_get(i);
    u16 global_index = sid_to_global_index(sid);
    acq_status_t *acq = &acq_status[global_index];

    /* Has an error occurred? */
    if (tracking_channel_error(i)) {
      log_warn_sid(sid, "error occurred, dropping");
      drop_channel(i);
      continue;
    }

    /* Is tracking masked? */
    if (track_mask[global_index]) {
      drop_channel(i);
      continue;
    }

    /* Give newly-initialized channels a chance to converge */
    if (tracking_channel_running_time_ms_get(i) < TRACK_INIT_T) {
      continue;
    }

    /* Is ephemeris or alert flag marked unhealthy?*/
    const ephemeris_t *e = ephemeris_get(sid);
    /* TODO: check alert flag */
    if (e->valid && !satellite_healthy(e)) {
      log_info_sid(sid, "unhealthy, dropping");
      drop_channel(i);
      acq->state = ACQ_PRN_UNHEALTHY;
      continue;
    }

    /* Do we not have nav bit sync yet? */
    if (!tracking_channel_bit_sync_resolved(i)) {
      drop_channel(i);
      continue;
    }

    /* Optimistic phase lock detector "unlocked" for a while? */
    /* TODO: This isn't doing much.  Use the pessimistic detector instead? */
    if (tracking_channel_ld_opti_unlocked_ms_get(i) > TRACK_DROP_UNLOCKED_T) {
      log_info_sid(sid, "PLL unlocked too long, dropping");
      drop_channel(i);
      continue;
    }

    /* CN0 below threshold for a while? */
    if (tracking_channel_cn0_drop_ms_get(i) > TRACK_DROP_CN0_T) {
      log_info_sid(sid, "low CN0 too long, dropping");
      drop_channel(i);
      continue;
    }

    /* Is satellite below our elevation mask? */
    if (tracking_channel_evelation_degrees_get(i) < elevation_mask) {
      log_info_sid(sid, "below elevation mask, dropping");
      drop_channel(i);
      /* Erase the tracking hint score, and any others it might have */
      memset(&acq->score, 0, sizeof(acq->score));
      continue;
    }
  }
}

s8 use_tracking_channel(u8 i)
{
  /* To use a channel's measurements in an SPP or RTK solution, we
     require the following conditions: */
  if (tracking_channel_running(i)
      /* Make sure no errors have occurred. */
      && !tracking_channel_error(i)
      /* Check SNR has been above threshold for the minimum time. */
      && (tracking_channel_cn0_useable_ms_get(i) > TRACK_SNR_THRES_COUNT)
      /* Satellite elevation is above the mask. */
      && (tracking_channel_evelation_degrees_get(i) >= elevation_mask)
      /* Pessimistic phase lock detector = "locked". */
      && (tracking_channel_ld_pess_locked_ms_get(i) > TRACK_USE_LOCKED_T)
      /* Some time has elapsed since the last tracking channel mode
       * change, to allow any transients to stabilize.
       * TODO: is this still necessary? */
      && (tracking_channel_last_mode_change_ms_get(i) > TRACK_STABILIZATION_T)
      /* Channel time of week has been decoded. */
      && (tracking_channel_tow_ms_get(i) != TOW_INVALID)
      /* Nav bit polarity is known, i.e. half-cycles have been resolved. */
      && tracking_channel_bit_polarity_resolved(i))
      /* TODO: Alert flag is not set */
      {
    /* Ephemeris must be valid, not stale. Satellite must be healthy.
       This also acts as a sanity check on the channel TOW.*/
    gps_time_t t = {
      /* TODO: the following makes the week number part of the
         TOW check tautological - see issue #475 */
      .wn = WN_UNKNOWN,
      .tow = 1e-3 * tracking_channel_tow_ms_get(i)
    };
    ephemeris_t *e = ephemeris_get(tracking_channel_sid_get(i));
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

/** Checks if tracking can be started for a given sid.
 *
 * \param sid Signal ID to check.
 * \retval true sid tracking can be started.
 * \retval false sid tracking cannot be started.
 */
bool tracking_startup_ready(gnss_signal_t sid)
{
  u16 global_index = sid_to_global_index(sid);
  acq_status_t *acq = &acq_status[global_index];
  return (acq->state == ACQ_PRN_ACQUIRING) && (!acq->masked);
}

/** Queue a request to start up tracking and decoding for the specified sid.
 *
 * \note This function is thread-safe and non-blocking.
 *
 * \param startup_params    Struct containing startup parameters.
 *
 * \return true if the request was successfully submitted, false otherwise.
 */
bool tracking_startup_request(const tracking_startup_params_t *startup_params)
{
  bool result = false;
  if(chMtxTryLock(&tracking_startup_mutex))
  {
    result = tracking_startup_fifo_write(&tracking_startup_fifo,
                                         startup_params);

    chMtxUnlock(&tracking_startup_mutex);
  }

  return result;
}

/** Read tracking startup requests from the FIFO and attempt to start
 * tracking and decoding.
 */
static void manage_tracking_startup(void)
{
  tracking_startup_params_t startup_params;
  while(tracking_startup_fifo_read(&tracking_startup_fifo, &startup_params)) {

    acq_status_t *acq = &acq_status[sid_to_global_index(startup_params.sid)];

    /* Make sure the SID is not already tracked. */
    if (acq->state == ACQ_PRN_TRACKING) {
      continue;
    }

    /* Make sure a tracking channel and a decoder channel are available */
    u8 chan = manage_track_new_acq(startup_params.sid);
    if (chan == MANAGE_NO_CHANNELS_FREE) {

      /* No channels are free to accept our new satellite :( */
      /* TODO: Perhaps we can try to warm start this one
       * later using another fine acq.
       */
      if (startup_params.cn0_init > ACQ_RETRY_THRESHOLD) {
        acq->score[ACQ_HINT_PREV_ACQ] =
            SCORE_ACQ + (startup_params.cn0_init - ACQ_THRESHOLD);
        acq->dopp_hint_low = startup_params.carrier_freq - ACQ_FULL_CF_STEP;
        acq->dopp_hint_high = startup_params.carrier_freq + ACQ_FULL_CF_STEP;
      }

      continue;
    }

    /* Start the tracking channel */
    if(!tracker_channel_init(chan, startup_params.sid,
                             startup_params.sample_count,
                             startup_params.code_phase,
                             startup_params.carrier_freq,
                             startup_params.chips_to_correlate,
                             startup_params.cn0_init,
                             TRACKING_ELEVATION_UNKNOWN)) {
      log_error("tracker channel init failed");
    }
    /* TODO: Initialize elevation from ephemeris if we know it precisely */

    /* Start the decoder channel */
    if (!decoder_channel_init(chan, startup_params.sid)) {
      log_error("decoder channel init failed");
    }

    /* Change state to TRACKING */
    acq->state = ACQ_PRN_TRACKING;
  }
}

/** Initialize a tracking_startup_fifo_t struct.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 */
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo)
{
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Write data to a tracking startup FIFO.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool tracking_startup_fifo_write(tracking_startup_fifo_t *fifo,
                                        const tracking_startup_params_t *
                                        element)
{
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) < TRACKING_STARTUP_FIFO_SIZE) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(&fifo->elements[fifo->write_index &
                           TRACKING_STARTUP_FIFO_INDEX_MASK],
           element, sizeof(tracking_startup_params_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->write_index++;
    return true;
  }

  return false;
}

/** Read pending data from a tracking startup FIFO.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element)
{
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) > 0) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(element,
           &fifo->elements[fifo->read_index &
                           TRACKING_STARTUP_FIFO_INDEX_MASK],
           sizeof(tracking_startup_params_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** \} */
