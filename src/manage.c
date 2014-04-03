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
#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/sbp.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "board/nap/acq_channel.h"
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

acq_prn_t acq_prn_param[32];
almanac_t almanac[32];

acq_manage_t acq_manage;

sbp_msg_callbacks_node_t almanac_callback_node;
void almanac_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  almanac_t *new_almanac = (almanac_t*)msg;

  printf("Received alamanc for PRN %02d\n", new_almanac->prn);
  memcpy(&almanac[new_almanac->prn-1], new_almanac, sizeof(almanac_t));

  int fd = cfs_open("almanac", CFS_WRITE);
  if (fd != -1) {
    cfs_seek(fd, (new_almanac->prn-1)*sizeof(almanac_t), CFS_SEEK_SET);
    if (cfs_write(fd, new_almanac, sizeof(almanac_t)) != sizeof(almanac_t))
      printf("Error writing to almanac file\n");
    else
      printf("Saved almanac to flash\n");
    cfs_close(fd);
  } else {
    printf("Error opening almanac file\n");
  }
}

static WORKING_AREA(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static msg_t manage_acq_thread(void *arg)
{
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(100);
    manage_acq();
  }

  return 0;
}

void manage_acq_setup()
{
  for (u8 prn=0; prn<32; prn++) {
    acq_prn_param[prn].state = ACQ_PRN_UNTRIED;
    acq_prn_param[prn].score = 0;
  }

  int fd = cfs_open("almanac", CFS_READ);
  if (fd != -1) {
    cfs_read(fd, almanac, 32*sizeof(almanac_t));
    printf("Loaded almanac from flash\n");
    cfs_close(fd);
  } else {
    printf("No almanac file present in flash, create an empty one\n");
    cfs_coffee_reserve("almanac", 32*sizeof(almanac_t));
    cfs_coffee_configure_log("almanac", 256, sizeof(almanac_t));

    for (u8 prn=0; prn<32; prn++) {
      almanac[prn].valid = 0;
    }
  }

  sbp_register_cbk(
    MSG_ALMANAC,
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
  gps_time_t t;

  if (time_quality != TIME_UNKNOWN)
    t = get_current_time();

  for (u8 prn=0; prn<32; prn++) {
    if (!almanac[prn].valid ||
        time_quality == TIME_UNKNOWN ||
        position_quality == POSITION_UNKNOWN) {
      /* No almanac or position/time information, give it the benefit of the
       * doubt. */
      acq_prn_param[prn].score = 0;
    } else {
      calc_sat_az_el_almanac(&almanac[prn], t.tow, t.wn-1024, position_solution.pos_ecef, &az, &el);
      acq_prn_param[prn].score = (s8)(el/D2R);

      gps_time_t toa;
      toa.wn = almanac[prn].week + 1024;
      toa.tow = almanac[prn].toa;

      double dt = fabs(gpsdifftime(t, toa));

      if (time_quality == TIME_GUESS ||
          position_quality == POSITION_GUESS ||
          dt > 2*24*3600) {
        /* Don't exclude other sats if our time is just a guess or our almanac
         * is old. */
        if (acq_prn_param[prn].score < 0)
          acq_prn_param[prn].score = 0;
      }
    }
  }
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
void manage_acq()
{
  nap_track_n_channels = 9;
  switch (acq_manage.state) {
    default:
    case ACQ_MANAGE_START: {
      /* Check if there are tracking channels free first. */
      u8 i;
      for (i=0; i<nap_track_n_channels; i++) {
        if (tracking_channel[i].state == TRACKING_DISABLED)
          break;
      }
      if (i == nap_track_n_channels)
        /* No tracking channels free :( */
        break;

      /* Tracking channel is free, decide which PRN
       * to try and then start it acquiring. */
      s8 best_prn = -1;
      s8 best_score = -1;
      manage_calc_scores();
      for (u8 prn=0; prn<32; prn++) {
        if ((acq_prn_param[prn].state != ACQ_PRN_TRACKING) &&
            (acq_prn_param[prn].state != ACQ_PRN_TRIED) &&
            (acq_prn_param[prn].score > best_score)) {
          best_prn = prn;
          best_score = acq_prn_param[prn].score;
        }
      }

      if (best_score < 0) {
        /* No good satellites right now. Set all back to untried and try again
         * later. */
        for (u8 prn=0; prn<32; prn++) {
          if (acq_prn_param[prn].state == ACQ_PRN_TRIED)
            acq_prn_param[prn].state = ACQ_PRN_UNTRIED;
        }
        break;
      }

      /* We have our PRN chosen, now load some fresh data
       * into the acquisition ram on the Swift NAP for
       * an initial coarse acquisition.
       */
      /*printf("Acq choosing PRN: %d\n", best_prn+1);*/
      acq_manage.prn = best_prn;
      acq_prn_param[best_prn].state = ACQ_PRN_ACQUIRING;
      acq_manage.state = ACQ_MANAGE_LOADING_COARSE;
      acq_manage.coarse_timer_count = nap_timing_count() + 20000;
      acq_schedule_load(acq_manage.coarse_timer_count);
      break;
    }

    case ACQ_MANAGE_LOADING_COARSE:
      /* TODO: Loading should be part of the acq code not the manage code. */
      if ((u32)nap_timing_count() - acq_manage.coarse_timer_count > 2*SAMPLE_FREQ) {
        printf("Coarse loading timeout %u %u\n", (unsigned int)nap_timing_count(), (unsigned int)acq_manage.coarse_timer_count);
        acq_manage.state = ACQ_MANAGE_START;
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_UNTRIED;
      }
      /* Wait until we are done loading. */
      acq_wait_load_done();

      /* Done loading, now lets set that coarse acquisition going. */
      nap_acq_code_wr_blocking(acq_manage.prn);
      if (almanac[acq_manage.prn].valid && time_quality == TIME_COARSE) {
        gps_time_t t = rx2gpstime(acq_manage.coarse_timer_count);

        double dopp = -calc_sat_doppler_almanac(&almanac[acq_manage.prn], t.tow, t.wn, position_solution.pos_ecef);
        /* TODO: look into accuracy of prediction and possibilities for
         * improvement, e.g. use clock bias estimated by PVT solution. */
        /*printf("Expecting PRN %02d @ %.1f\n", acq_manage.prn+1, dopp);*/
        acq_start(acq_manage.prn, 0, 1023, dopp-4000, dopp+4000, ACQ_FULL_CF_STEP);
      } else {
        acq_start(acq_manage.prn, 0, 1023,
            ACQ_FULL_CF_MIN,
            ACQ_FULL_CF_MAX,
            ACQ_FULL_CF_STEP);
      }
      acq_manage.state = ACQ_MANAGE_RUNNING_COARSE;
      break;

    case ACQ_MANAGE_RUNNING_COARSE:
      /* Wait until we are done acquiring. */
      if (!acq_get_done())
        break;
      /* Done with the coarse acquisition, check if we have found a
       * satellite, if so save the results and start the loading
       * for the fine acquisition. If not, start again choosing a
       * different PRN.
       */
      acq_get_results(&acq_manage.coarse_cp,
                      &acq_manage.coarse_cf,
                      &acq_manage.coarse_snr);
      printf("PRN %d coarse @ %d Hz, %d SNR\n", acq_manage.prn + 1,
                                    (int)acq_manage.coarse_cf,
                                    (int)acq_manage.coarse_snr);
      if (acq_manage.coarse_snr < ACQ_THRESHOLD) {
        /* Didn't find the satellite :( */
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRIED;
        acq_manage.state = ACQ_MANAGE_START;
        break;
      }
      /* Looks like we have a winner! */
      acq_manage.state = ACQ_MANAGE_LOADING_FINE;
      acq_manage.fine_timer_count = nap_timing_count() + 20000;
      acq_schedule_load(acq_manage.fine_timer_count);
      break;

    case ACQ_MANAGE_LOADING_FINE:
      if ((u32)nap_timing_count() - acq_manage.fine_timer_count > 2*SAMPLE_FREQ) {
        printf("Fine loading timeout %u %u\n", (unsigned int)nap_timing_count(), (unsigned int)acq_manage.fine_timer_count);
        acq_manage.state = ACQ_MANAGE_START;
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_UNTRIED;
      }
      /* Wait until we are done loading. */
      acq_wait_load_done();

      /* Done loading, now lets set the fine acquisition going. */
      float fine_cp = propagate_code_phase(
                        acq_manage.coarse_cp,
                        acq_manage.coarse_cf,
                        acq_manage.fine_timer_count - acq_manage.coarse_timer_count
                      );
      acq_start(acq_manage.prn,
                fine_cp-ACQ_FINE_CP_WIDTH,
                fine_cp+ACQ_FINE_CP_WIDTH,
                acq_manage.coarse_cf-ACQ_FINE_CF_WIDTH,
                acq_manage.coarse_cf+ACQ_FINE_CF_WIDTH, ACQ_FINE_CF_STEP);
      acq_manage.state = ACQ_MANAGE_RUNNING_FINE;
      break;

    case ACQ_MANAGE_RUNNING_FINE: {
      /* Wait until we are done acquiring. */
      if (!acq_get_done())
        break;
      /* Fine acquisition done, check if we have the satellite still,
       * if so then transition to tracking, otherwise start again with
       * a different PRN.
       */
      float fine_cp, fine_cf;
      acq_get_results(&fine_cp, &fine_cf, &acq_manage.fine_snr);
      printf("PRN %d Fine @ %+.0f Hz,  %.1f SNR\n", acq_manage.prn + 1,
                                  fine_cf,
                                  acq_manage.fine_snr);
      /* BELOW REMOVED - if we found it in coarse then we'll consider it acquired. */
      // TODO: Change SNR calculation so it is valid for fine then reenable this.
      //if (acq_manage.fine_snr < ACQ_THRESHOLD) {
      //  /* Didn't find the satellite :( */
      //  acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRIED;
      //  acq_manage.state = ACQ_MANAGE_START;
      //  break;
      //}
      u8 chan = manage_track_new_acq(acq_manage.fine_snr);
      if (chan == MANAGE_NO_CHANNELS_FREE) {
        /* No channels are free to accept our new satellite :( */
        /* TODO: Perhaps we can try to warm start this one
         * later using another fine acq.
         */
        printf("No channels free :(\n");
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRIED;
        acq_manage.state = ACQ_MANAGE_START;
        break;
      }
      /* Transition to tracking. */
      u32 track_count = nap_timing_count() + 20000;
      float track_cp = propagate_code_phase(fine_cp, fine_cf, track_count - acq_manage.fine_timer_count);

      // Contrive for the timing strobe to occur at or close to a PRN edge (code phase = 0)
      track_count += 16*(1023.0-track_cp)*(1.0 + fine_cf / GPS_L1_HZ);

      tracking_channel_init(chan, acq_manage.prn, fine_cf, track_count);
      acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRACKING;
      acq_manage.state = ACQ_MANAGE_START;
      break;
    }
    case ACQ_MANAGE_DISABLED:
      /* Do nothing. */
      break;
  }
}

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \param snr SNR of the acquisition.
 * \return Index of first unused tracking channel.
 */
u8 manage_track_new_acq(float snr __attribute__((unused)))
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

static WORKING_AREA(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
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
  chThdCreateStatic(
      wa_manage_track_thread,
      sizeof(wa_manage_track_thread),
      MANAGE_TRACK_THREAD_PRIORITY,
      manage_track_thread, NULL
  );
}

/** Disable any tracking channel whose SNR is below a certain margin. */
void manage_track()
{
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (tracking_channel[i].state == TRACKING_RUNNING) {
      if (tracking_channel_snr(i) < TRACK_THRESHOLD) {
        if (tracking_channel[i].update_count > TRACK_SNR_INIT_COUNT &&
            tracking_channel[i].update_count - tracking_channel[i].snr_threshold_count > TRACK_SNR_THRES_COUNT) {
          /* This tracking channel has lost its satellite. */
          printf("Disabling channel %d\n", i);
          tracking_channel_disable(i);
          acq_prn_param[tracking_channel[i].prn].state = ACQ_PRN_TRIED;
        }
      } else {
        tracking_channel[i].snr_threshold_count = tracking_channel[i].update_count;
      }
    }
  }
}

/** \} */
