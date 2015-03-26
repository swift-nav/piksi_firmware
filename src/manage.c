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

/** \defgroup manage Manage
 * Manage acquisition and tracking.
 * Manage how acquisition searches are performed, with data from almanac if
 * available. Transition from acquisition search to initializization of an
 * available tracking channel when a satellite is successfully found. Disable
 * tracking channels that have lost lock on their satellites.
 * \{ */

/** Different hints on satellite info to aid the acqusition */
/* XXX add me **/
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
    ACQ_PRN_UNTRIED,
    ACQ_PRN_TRIED,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING
  } state;                 /**< Management status of PRN. */
  u16 score[ACQ_HINT_NUM]; /**< Acquisition preference of PRN. */
  float dopp_hint_low;     /**< Low bound of doppler search hint. */
  float dopp_hint_high;    /**< High bound of doppler search hint. */
} acq_prn_t;
acq_prn_t acq_prn_param[32];

almanac_t almanac[32];

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

static WORKING_AREA_CCM(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static msg_t manage_acq_thread(void *arg)
{
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */
  (void)arg;
  chRegSetThreadName("manage acq");
  while (TRUE) {
    manage_acq();
  }

  return 0;
}

void manage_acq_setup()
{
  for (u8 prn=0; prn<32; prn++) {
    acq_prn_param[prn].state = ACQ_PRN_UNTRIED;
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

  chThdCreateStatic(
      wa_manage_acq_thread,
      sizeof(wa_manage_acq_thread),
      MANAGE_ACQ_THREAD_PRIORITY,
      manage_acq_thread, NULL
  );
}

static void manage_calc_scores(void)
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
      acq_prn_param[prn].score[ACQ_HINT_ALMANAC] = el > 0 ? (u8)(el/D2R) : 0;
      acq_prn_param[prn].dopp_hint_low = dopp - 4000;
      acq_prn_param[prn].dopp_hint_high = dopp + 4000;
    }
  }
}

static u8 best_prn(void)
{
  s8 best_prn = -1;
  s16 best_score = -1;
  for (u8 prn=0; prn<32; prn++) {

    u16 score = 0;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      score += acq_prn_param[prn].score[hint];

    if ((acq_prn_param[prn].state != ACQ_PRN_TRACKING) &&
        (acq_prn_param[prn].state != ACQ_PRN_TRIED) &&
        (score > best_score)) {
      best_prn = prn;
      best_score = score;
    }
  }

  if (best_score < 0) {
    /* No good satellites right now. Set all back to untried and try again
     * later. */
    for (u8 prn=0; prn<32; prn++) {
      if (acq_prn_param[prn].state == ACQ_PRN_TRIED)
        acq_prn_param[prn].state = ACQ_PRN_UNTRIED;
    }
    log_info("acq: restarting PRN search\n");
    manage_calc_scores();
    return -1;
  }
  return best_prn;
}

/* Force a focus on specific satellites from Rover/Base observations

RTK relies on a common set of measurements, have the receivers focus search
efforts on satellites both are likely to be able to see. Receiver will need
to be sufficiently close for RTK to function, and so should have a similar
view of the constellation, even if obstructed this may change if the receivers
move or the satellite arcs across the sky.

I haven't tested how this behaves in all conditions, but should be self limiting
in that satellites that roll out of view will stop getting promoted, and spare
channels will get tasked with general hunting tasks. Satellites not in view to
both due to big separation aren't going to hurt anyways.

Prior to this flagging my Piksi's would perhaps both see 7 satellites, of these
only 5 would be in common. By sharing viable/visibles birds mutually between the
receivers we now have 9 common measurements to leverage into the RTK solution.

This could be augmented further by sharing ephemeris/alamanc, here the rapidity of
just tagging the PRN is a cheap win-win.

cturvey 10-Feb-2015
*/

void manage_prod_acq(u8 prn)
{
  if (prn >= 32) /* check range */
    return;

  acq_prn_param[prn].score[ACQ_HINT_OBS] = 90; /* 90 Degrees, want this to be a priority */
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
static void manage_acq()
{
  /* Decide which PRN to try and then start it acquiring. */
  u8 prn = best_prn();
  if (prn == (u8)-1)
    return;

  u32 timer_count;
  float snr, cp, cf;

  acq_set_prn(prn);

  /* We have our PRN chosen, now load some fresh data
   * into the acquisition ram on the Swift NAP for
   * an initial coarse acquisition.
   */
  acq_prn_param[prn].state = ACQ_PRN_ACQUIRING;
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
  acq_get_results(&cp, &cf, &snr);
  /* Send result of an acquisition to the host. */
  acq_send_result(prn, snr, cp, cf);
  if (snr < ACQ_THRESHOLD) {
    /* Didn't find the satellite :( */
    float dilute = (acq_prn_param[prn].dopp_hint_high -
                    acq_prn_param[prn].dopp_hint_low) / 2;
    acq_prn_param[prn].dopp_hint_high =
        MIN(acq_prn_param[prn].dopp_hint_high + dilute, ACQ_FULL_CF_MAX);
    acq_prn_param[prn].dopp_hint_low =
        MAX(acq_prn_param[prn].dopp_hint_low - dilute, ACQ_FULL_CF_MIN);
    acq_prn_param[prn].score[ACQ_HINT_ACQ] = 0;
    acq_prn_param[prn].state = ACQ_PRN_TRIED;
    return;
  }

  log_info("acq: PRN %d found @ %d Hz, %d SNR\n", prn + 1, (int)cf, (int)snr);

  u8 chan = manage_track_new_acq();
  if (chan == MANAGE_NO_CHANNELS_FREE) {
    /* No channels are free to accept our new satellite :( */
    /* TODO: Perhaps we can try to warm start this one
     * later using another fine acq.
     */
    log_info("No channels free :(\n");
    if (snr > ACQ_RETRY_THRESHOLD) {
      acq_prn_param[prn].score[ACQ_HINT_ACQ] = MIN(snr, 127);
      acq_prn_param[prn].dopp_hint_low = cf - ACQ_FULL_CF_STEP;
      acq_prn_param[prn].dopp_hint_high = cf + ACQ_FULL_CF_STEP;
      acq_prn_param[prn].state = ACQ_PRN_UNTRIED;
    } else {
      acq_prn_param[prn].state = ACQ_PRN_TRIED;
    }
    return;
  }
  /* Transition to tracking. */
  u32 track_count = nap_timing_count() + 20000;
  cp = propagate_code_phase(cp, cf, track_count - timer_count);

  // Contrive for the timing strobe to occur at or close to a PRN edge (code phase = 0)
  track_count += 16*(1023.0-cp)*(1.0 + cf / GPS_L1_HZ);

  tracking_channel_init(chan, prn, cf, track_count, snr);
  acq_prn_param[prn].state = ACQ_PRN_TRACKING;
  nap_timing_strobe_wait(100);
}

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \param snr SNR of the acquisition.
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

static WORKING_AREA_CCM(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage track");
  while (TRUE) {
    chThdSleepMilliseconds(200);
    DO_EVERY(5,
      manage_track();
      nmea_gpgsa(tracking_channel, 0);
    );
    tracking_send_state();
  }

  return 0;
}

void manage_track_setup()
{
  initialize_lock_counters();

  chThdCreateStatic(
      wa_manage_track_thread,
      sizeof(wa_manage_track_thread),
      MANAGE_TRACK_THREAD_PRIORITY,
      manage_track_thread, NULL
  );
}

/** Disable any tracking channel whose SNR is below a certain margin. */
static void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {
    tracking_channel_t *ch = &tracking_channel[i];
    if (ch->state != TRACKING_RUNNING)
      continue;

    if (tracking_channel_snr(i) < TRACK_THRESHOLD) {
      ch->snr_below_threshold_count = ch->update_count;
      if (ch->update_count > TRACK_SNR_INIT_COUNT &&
          ch->update_count - ch->snr_above_threshold_count >
            TRACK_SNR_THRES_COUNT) {
        /* This tracking channel has lost its satellite. */
        log_info("Disabling channel %d\n", i);
        tracking_channel_disable(i);
        if (ch->snr_above_threshold_count > TRACK_SNR_THRES_COUNT) {
          acq_prn_param[ch->prn].score[ACQ_HINT_TRACK] = 90;
          float cf = ch->carrier_freq;
          acq_prn_param[ch->prn].dopp_hint_low = cf - ACQ_FULL_CF_STEP;
          acq_prn_param[ch->prn].dopp_hint_high = cf + ACQ_FULL_CF_STEP;
        }
        acq_prn_param[ch->prn].state = ACQ_PRN_TRIED;
      }
    } else {
      ch->snr_above_threshold_count = ch->update_count;
    }
  }
}

extern ephemeris_t es[32];
s8 use_tracking_channel(u8 i)
{
  return (tracking_channel[i].state == TRACKING_RUNNING)
      && (es[tracking_channel[i].prn].valid == 1)
      && (es[tracking_channel[i].prn].healthy == 1)
      && (tracking_channel[i].update_count
            - tracking_channel[i].snr_below_threshold_count
            > TRACK_SNR_THRES_COUNT)
      && (tracking_channel[i].TOW_ms > 0);
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
