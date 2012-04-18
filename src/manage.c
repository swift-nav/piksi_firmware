/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>

#include "acq.h"
#include "track.h"
#include "manage.h"
#include "debug.h"

acq_prn_t acq_prn_param[32] = {
  [0 ... 31] = { .state = ACQ_PRN_SKIP },
};

acq_manage_t acq_manage;

msg_callbacks_node_t acq_setup_callback_node;
void acq_setup_callback(u8 buff[])
{
  acq_prn_t *acq_prn_param_new = (acq_prn_t*)buff;

  /* Copy PRN parameters from the setup message for all the PRNs
   * that are not either tracking or acquiring.
   */
  for (u8 prn=0; prn<32; prn++) {
    if (acq_prn_param[prn].state != ACQ_PRN_ACQUIRING
        && acq_prn_param[prn].state != ACQ_PRN_TRACKING) {
      /*printf("Setting PRN %02d, %d %d ... %d\n", prn+1,*/
          /*acq_prn_param_new[prn].state,*/
          /*acq_prn_param_new[prn].carrier_freq_min,*/
          /*acq_prn_param_new[prn].carrier_freq_max*/
      /*);*/
      memcpy(&acq_prn_param[prn], &acq_prn_param_new[prn], sizeof(acq_prn_t));
    }
  }
}

void manage_acq_setup()
{
  for (u8 prn=0; prn<32; prn++) {
    acq_prn_param[prn].carrier_freq_min = ACQ_FULL_CF_MIN;
    acq_prn_param[prn].carrier_freq_max = ACQ_FULL_CF_MAX;
  }
  debug_register_callback(MSG_ACQ_SETUP, &acq_setup_callback, &acq_setup_callback_node);
}

void manage_acq()
{
  switch (acq_manage.state) {
    default:
    case ACQ_MANAGE_START: {
      /* Check if there are tracking channels free first. */
      u8 i;
      for (i=0; i<TRACK_N_CHANNELS; i++) {
        if (tracking_channel[i].state == TRACKING_DISABLED)
          break;
      }
      if (i == TRACK_N_CHANNELS)
        /* No tracking channels free :( */
        break;

      /* Acquisition channel is free, decide which PRN
       * to try and then start it acquiring.
       */
      u8 prn;
      for (prn=0; prn<32; prn++) {
        if (acq_prn_param[prn].state == ACQ_PRN_UNTRIED)
          break;
      }

      if (prn == 32) {
        /* Didn't find any untried PRNs, reset all tried
         * PRNs to untried and then pick the first one.
         */
        for (u8 n=0; n<32; n++) {
          if (acq_prn_param[n].state == ACQ_PRN_TRIED) {
            /* Choose the first tried PRN to be our next
             * to acquire.
             */
            if (prn == 32)
              prn = n;
            acq_prn_param[n].state = ACQ_PRN_UNTRIED;
          }
        }
      }

      /* If we still can't find any PRNs to try then we must be
       * tracking them all? Lets try again in a bit.
       */
      if (prn == 32)
        break;

      /* We have our PRN chosen, now load some fresh data
       * into the acquisition ram on the Swift NAP for
       * an initial coarse acquisition.
       */
     // printf("Acq choosing PRN: %d\n", prn+1);
      acq_manage.prn = prn;
      acq_prn_param[prn].state = ACQ_PRN_ACQUIRING;
      acq_manage.state = ACQ_MANAGE_LOADING_COARSE;
      acq_manage.coarse_timer_count = timing_count() + 1000;
      acq_schedule_load(acq_manage.coarse_timer_count);
      break;
    }

    case ACQ_MANAGE_LOADING_COARSE:
      if (timing_count() - acq_manage.coarse_timer_count > 2*SAMPLE_FREQ) {
        printf("Coarse loading error %u %u\n", (unsigned int)timing_count(), (unsigned int)acq_manage.coarse_timer_count);
      }
      /* Wait until we are done loading. */
      if (!acq_get_load_done())
        break;
      /* Done loading, now lets set that coarse acquisition going. */
      acq_write_code_blocking(acq_manage.prn);
      acq_start(acq_manage.prn, 0, 1023,
          acq_prn_param[acq_manage.prn].carrier_freq_min,
          acq_prn_param[acq_manage.prn].carrier_freq_max,
          ACQ_FULL_CF_STEP);
      acq_manage.state = ACQ_MANAGE_RUNNING_COARSE;

      /* Reset the carrier frequency window for the next pass. */
      acq_prn_param[acq_manage.prn].carrier_freq_min = ACQ_FULL_CF_MIN;
      acq_prn_param[acq_manage.prn].carrier_freq_max = ACQ_FULL_CF_MAX;
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
      printf("PRN %d coarse @ %+.0f Hz, %.1f SNR\n", acq_manage.prn + 1,
                                    acq_manage.coarse_cf,
                                    acq_manage.coarse_snr);
      if (acq_manage.coarse_snr < ACQ_THRESHOLD) {
        /* Didn't find the satellite :( */
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRIED;
        acq_manage.state = ACQ_MANAGE_START;
        break;
      }
      /* Looks like we have a winner! */
      acq_manage.state = ACQ_MANAGE_LOADING_FINE;
      acq_manage.fine_timer_count = timing_count() + 1000;
      acq_schedule_load(acq_manage.fine_timer_count);
      break;

    case ACQ_MANAGE_LOADING_FINE:
      if (timing_count() - acq_manage.fine_timer_count > 2*SAMPLE_FREQ) {
        printf("Fine loading error %u %u\n", (unsigned int)timing_count(), (unsigned int)acq_manage.fine_timer_count);
      }
      /* Wait until we are done loading. */
      if (!acq_get_load_done())
        break;
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
      // BELOW REMOVED - if we found it in coarse then we'll consider it acquired.
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
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_UNTRIED;
        acq_manage.state = ACQ_MANAGE_START;
        break;
      }
      /* Transition to tracking. */
      u32 track_count = timing_count() + 20000;
      float track_cp = propagate_code_phase(fine_cp, fine_cf, track_count - acq_manage.fine_timer_count);

      // Contrive for the timing strobe to occur at or close to a PRN edge (code phase = 0)
      track_count += 16*(1023.0-track_cp)*(1.0 + fine_cf / L1_HZ);

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

u8 manage_track_new_acq(float snr __attribute__((unused)))
{
  /* Decide which (if any) tracking channel to put
   * a newly acquired satellite into.
   */
  for (u8 i=0; i<TRACK_N_CHANNELS-1; i++) {
    if (tracking_channel[i].state == TRACKING_DISABLED) {
      return i;
    }
  }

  return MANAGE_NO_CHANNELS_FREE;
}

void manage_track()
{
  for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
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
