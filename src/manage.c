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

#include "acq.h"
#include "track.h"
#include "manage.h"

acq_prn_t acq_prn_param[32] = {
  [19] = {.state = ACQ_PRN_UNTRIED},
  [22] = {.state = ACQ_PRN_UNTRIED},
  [31] = {.state = ACQ_PRN_UNTRIED}
};

acq_manage_t acq_manage;

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
      printf("Acq choosing PRN: %d\n", prn+1);
      acq_manage.prn = prn;
      acq_prn_param[prn].state = ACQ_PRN_ACQUIRING;
      acq_manage.state = ACQ_MANAGE_LOADING_COARSE;
      acq_manage.coarse_timer_count = timing_count() + 1000;
      acq_schedule_load(acq_manage.coarse_timer_count);
      break;
    }

    case ACQ_MANAGE_LOADING_COARSE:
      /* Wait until we are done loading. */
      if (!acq_get_load_done())
        break;
      /* Done loading, now lets set that coarse acquisition going. */
      acq_start(acq_manage.prn, 0, 1023, -7000, 7000, 300);
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
      printf("Coarse %f, %f, %f\n", acq_manage.coarse_cp,
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
                fine_cp-20,
                fine_cp+20,
                acq_manage.coarse_cf-300,
                acq_manage.coarse_cf+300, 100);
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
      printf("Fine %f, %f, %f\n", fine_cp,
                                  fine_cf,
                                  acq_manage.fine_snr);
      if (acq_manage.fine_snr < ACQ_THRESHOLD) {
        /* Didn't find the satellite :( */
        acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRIED;
        acq_manage.state = ACQ_MANAGE_START;
        break;
      }
      /* Transition to tracking. */
      u32 track_count = timing_count() + 20000;
      float track_cp = propagate_code_phase(fine_cp, fine_cf, track_count - acq_manage.fine_timer_count);
      tracking_channel_init(0, acq_manage.prn, track_cp, fine_cf, track_count);
      acq_prn_param[acq_manage.prn].state = ACQ_PRN_TRACKING;
      acq_manage.state = ACQ_MANAGE_START;
      break;
    }
  }
}

