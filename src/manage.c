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
  ACQ_HINT_ALMANAC,  /**< Almanac information. */
  ACQ_HINT_ACQ,      /**< Previous successful acqusition. */
  ACQ_HINT_TRACK,    /**< Previously tracked satellite. */
  ACQ_HINT_OBS,      /**< Observation from reference station. */

  ACQ_HINT_NUM
};

/** Status of acquisition for a particular PRN. */
typedef struct {
  enum {
    ACQ_PRN_SKIP = 0,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING,
    ACQ_PRN_UNHEALTHY
  } state;                 /**< Management status of PRN. */
  bool masked;             /**< Prevent acquisition. */
  u16 score[ACQ_HINT_NUM]; /**< Acquisition preference of PRN. */
  float dopp_hint_low;     /**< Low bound of doppler search hint. */
  float dopp_hint_high;    /**< High bound of doppler search hint. */
} acq_prn_t;
acq_prn_t acq_prn_param[32];

#define SCORE_DEFAULT 100
#define SCORE_ALMANAC 200
#define SCORE_ACQ     100
#define SCORE_TRACK   200
#define SCORE_OBS     200

#define ALMANAC_DOPPLER_WINDOW 4000

almanac_t almanac[32];
extern ephemeris_t es[32];

static float track_cn0_use_thres = 28.0;

static u8 manage_track_new_acq(void);
static void manage_acq(void);
static void manage_track(void);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  almanac_t *new_almanac = (almanac_t*)msg;

  log_info("Received alamanc for PRN %02d\n", new_almanac->prn);
  memcpy(&almanac[new_almanac->prn-1], new_almanac, sizeof(almanac_t));

  int fd = cfs_open("almanac", CFS_WRITE);
  if (fd != -1) {
    cfs_seek(fd, (new_almanac->prn-1)*sizeof(almanac_t), CFS_SEEK_SET);
    if (cfs_write(fd, new_almanac, sizeof(almanac_t)) != sizeof(almanac_t)) {
      log_error("Error writing to almanac file\n");
    } else {
      log_info("Saved almanac to flash\n");
    }
    cfs_close(fd);
  } else {
    log_error("Error opening almanac file\n");
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

  log_info("Mask for PRN %02d = 0x%02x\n", m->prn+1, m->mask);
  if (m->mask & MASK_ACQUISITION) {
    acq_prn_param[m->prn].masked = true;
  } else {
    acq_prn_param[m->prn].masked = false;
  }
  if (m->mask & MASK_TRACKING) {
    tracking_drop_satellite(m->prn);
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
  for (u8 prn=0; prn<32; prn++) {
    acq_prn_param[prn].state = ACQ_PRN_ACQUIRING;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      acq_prn_param[prn].score[hint] = 0;
    acq_prn_param[prn].dopp_hint_low = ACQ_FULL_CF_MIN;
    acq_prn_param[prn].dopp_hint_high = ACQ_FULL_CF_MAX;
  }

  int fd = cfs_open("almanac", CFS_READ);
  if (fd != -1) {
    cfs_read(fd, almanac, 32*sizeof(almanac_t));
    log_info("Loaded almanac from flash\n");
    cfs_close(fd);
  } else {
    log_info("No almanac file present in flash, create an empty one\n");
    cfs_coffee_reserve("almanac", 32*sizeof(almanac_t));
    cfs_coffee_configure_log("almanac", 256, sizeof(almanac_t));

    for (u8 prn=0; prn<32; prn++) {
      almanac[prn].valid = 0;
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

static void manage_calc_almanac_scores(void)
{
  double az, el;
  gps_time_t t = get_current_time();

  for (u8 prn=0; prn<32; prn++) {
    if (!almanac[prn].valid ||
        time_quality == TIME_UNKNOWN ||
        position_quality == POSITION_UNKNOWN) {
      /* No almanac or position/time information, give it the benefit of the
       * doubt. */
      acq_prn_param[prn].score[ACQ_HINT_ALMANAC] = 0;
    } else {
      calc_sat_az_el_almanac(&almanac[prn], t.tow, t.wn-1024, position_solution.pos_ecef, &az, &el);
      float dopp = -calc_sat_doppler_almanac(&almanac[prn], t.tow, t.wn,
                                             position_solution.pos_ecef);
      acq_prn_param[prn].score[ACQ_HINT_ALMANAC] =
            el > 0 ? (u16)(SCORE_ALMANAC * 2 * el / M_PI) : 0;
      acq_prn_param[prn].dopp_hint_low = dopp - ALMANAC_DOPPLER_WINDOW;
      acq_prn_param[prn].dopp_hint_high = dopp + ALMANAC_DOPPLER_WINDOW;
    }
  }
}

static u8 choose_prn(void)
{
  u32 total_score = 0;

  manage_calc_almanac_scores();

  for (u8 prn=0; prn<32; prn++) {
    if ((acq_prn_param[prn].state != ACQ_PRN_ACQUIRING) ||
         acq_prn_param[prn].masked)
      continue;

    total_score += SCORE_DEFAULT;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
      total_score += acq_prn_param[prn].score[hint];
    }
  }

  u32 pick = random_int() % total_score;

  for (u8 prn=0; prn<32; prn++) {
    if ((acq_prn_param[prn].state != ACQ_PRN_ACQUIRING) ||
         acq_prn_param[prn].masked)
      continue;

    u32 sat_score = SCORE_DEFAULT;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      sat_score += acq_prn_param[prn].score[hint];
    if (pick < sat_score) {
      return prn;
    } else {
      pick -= sat_score;
    }
  }

  log_error("Failed to pick a sat for acquisition!\n");

  return -1;
}

/** Hint acqusition at satellites observed by peer.

RTK relies on a common set of measurements, have the receivers focus search
efforts on satellites both are likely to be able to see. Receiver will need
to be sufficiently close for RTK to function, and so should have a similar
view of the constellation, even if obstructed this may change if the receivers
move or the satellite arcs across the sky.

cturvey 10-Feb-2015
*/
void manage_set_obs_hint(u8 prn)
{
  if (prn >= 32) /* check range */
    return;

  acq_prn_param[prn].score[ACQ_HINT_OBS] = SCORE_OBS;
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
static void manage_acq()
{
  /* Decide which PRN to try and then start it acquiring. */
  u8 prn = choose_prn();
  if (prn == (u8)-1)
    return;

  u32 timer_count;
  float cn0, cp, cf;

  acq_set_prn(prn);

  /* We have our PRN chosen, now load some fresh data
   * into the acquisition ram on the Swift NAP for
   * an initial coarse acquisition.
   */
  do {
    timer_count = nap_timing_count() + 20000;
    /* acq_load could timeout if we're preempted and miss the timing strobe */
  } while (!acq_load(timer_count));

  acq_search(acq_prn_param[prn].dopp_hint_low,
             acq_prn_param[prn].dopp_hint_high,
             ACQ_FULL_CF_STEP);

  /* Done with the coarse acquisition, check if we have found a
   * satellite, if so save the results and start the loading
   * for the fine acquisition. If not, start again choosing a
   * different PRN.
   */
  acq_get_results(&cp, &cf, &cn0);
  /* Send result of an acquisition to the host. */
  acq_send_result(prn, cn0, cp, cf);
  if (cn0 < ACQ_THRESHOLD) {
    /* Didn't find the satellite :( */
    /* Double the size of the doppler search space for next time. */
    float dilute = (acq_prn_param[prn].dopp_hint_high -
                    acq_prn_param[prn].dopp_hint_low) / 2;
    acq_prn_param[prn].dopp_hint_high =
        MIN(acq_prn_param[prn].dopp_hint_high + dilute, ACQ_FULL_CF_MAX);
    acq_prn_param[prn].dopp_hint_low =
        MAX(acq_prn_param[prn].dopp_hint_low - dilute, ACQ_FULL_CF_MIN);
    /* Decay hint scores */
    for (u8 i = 0; i < ACQ_HINT_NUM; i++)
      acq_prn_param[prn].score[i] = (acq_prn_param[prn].score[i] * 3) / 4;
    /* Reset hint score for acuisition. */
    acq_prn_param[prn].score[ACQ_HINT_ACQ] = 0;
    return;
  }

  u8 chan = manage_track_new_acq();
  if (chan == MANAGE_NO_CHANNELS_FREE) {
    /* No channels are free to accept our new satellite :( */
    /* TODO: Perhaps we can try to warm start this one
     * later using another fine acq.
     */
    if (cn0 > ACQ_RETRY_THRESHOLD) {
      acq_prn_param[prn].score[ACQ_HINT_ACQ] = SCORE_ACQ + (cn0 - ACQ_THRESHOLD);
      acq_prn_param[prn].dopp_hint_low = cf - ACQ_FULL_CF_STEP;
      acq_prn_param[prn].dopp_hint_high = cf + ACQ_FULL_CF_STEP;
    }
    return;
  }
  /* Transition to tracking. */
  u32 track_count = nap_timing_count() + 20000;
  cp = propagate_code_phase(cp, cf, track_count - timer_count);

  // Contrive for the timing strobe to occur at or close to a PRN edge (code phase = 0)
  track_count += 16*(1023.0-cp)*(1.0 + cf / GPS_L1_HZ);

  tracking_channel_init(chan, prn, cf, track_count, cn0);
  acq_prn_param[prn].state = ACQ_PRN_TRACKING;
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

/** Clear unhealthy flags after some time.  Flags are reset once per day. */
static void check_clear_unhealthy(void)
{
  static systime_t ticks;
  if (chTimeElapsedSince(ticks) < S2ST(24*60*60))
    return;

  ticks = chTimeNow();

  for (u8 prn=0; prn<32; prn++) {
    if (acq_prn_param[prn].state == ACQ_PRN_UNHEALTHY)
      acq_prn_param[prn].state = ACQ_PRN_ACQUIRING;
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
  if (ch->update_count > TRACK_REACQ_T) {
    acq_prn_param[ch->prn].score[ACQ_HINT_TRACK] = SCORE_TRACK;
    acq_prn_param[ch->prn].dopp_hint_low = ch->carrier_freq - ACQ_FULL_CF_STEP;
    acq_prn_param[ch->prn].dopp_hint_high = ch->carrier_freq + ACQ_FULL_CF_STEP;
  }
  acq_prn_param[ch->prn].state = ACQ_PRN_ACQUIRING;
}

/** Disable any tracking channel that has lost phase lock or is
    flagged unhealthy in ephem. */
static void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {

    tracking_channel_t *ch = &tracking_channel[i];
    /* Skip channels that aren't in use */
    if (ch->state != TRACKING_RUNNING ||
        /* Give newly-initialized channels a chance to converge */
        ch->update_count < TRACK_INIT_T)
      continue;

    /* Is ephemeris marked unhealthy? */
    if (es[ch->prn].valid && !es[ch->prn].healthy) {
      log_info("PRN%d unhealthy, dropping\n", ch->prn+1);
      drop_channel(i);
      acq_prn_param[ch->prn].state = ACQ_PRN_UNHEALTHY;
      continue;
    }

    /* Do we not have nav bit sync yet? */
    if (ch->nav_msg.bit_phase_ref == BITSYNC_UNSYNCED) {
      drop_channel(i);
      continue;
    }

    u32 uc = ch->update_count;

    /* Optimistic phase lock detector "unlocked" for a while? */
    if (uc - ch->ld_opti_locked_count > TRACK_DROP_UNLOCKED_T) {
      log_info("PRN%d PLL unlocked too long, dropping\n", ch->prn+1);
      drop_channel(i);
      continue;
    }

    /* CN0 below threshold for a while? */
    if (uc - ch->cn0_above_drop_thres_count > TRACK_DROP_CN0_T) {
      if (ch->cn0 >= 25.0) {
        log_error("PRN%d bogus 'low CN0 too long'\n", ch->prn+1);
      } else {
        log_info("PRN%d low CN0 too long, dropping\n", ch->prn+1);
        drop_channel(i);
      }
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
      /* Pessimistic phase lock detector = "locked". */
      && (ch->lock_detect.outp)
      /* Some time has elapsed since the last tracking channel mode
       * change, to allow any transients to stabilize.
       * TODO: is this still necessary? */
      && (ch->update_count - ch->mode_change_count > TRACK_STABILIZATION_T)
      /* Channel time of week has been decoded. */
      && (ch->TOW_ms != TOW_INVALID)
      /* Nav bit polarity is known, i.e. half-cycles have been resolved. */
      && (ch->nav_msg.bit_polarity != BIT_POLARITY_UNKNOWN)
      /* Estimated C/N0 is above some threshold */
      && (ch->cn0 > track_cn0_use_thres))
      {
    /* Ephemeris must be valid and not stale.
       This also acts as a sanity check on the channel TOW.*/
    gps_time_t t = {
      /* TODO: the following makes the week number part of the
         TOW check tautological - see issue #475 */
      .wn = es[ch->prn].toe.wn,
      .tow = 1e-3 * ch->TOW_ms
    };
    return ephemeris_good(&es[ch->prn], t);
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
