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
#include <math.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "cw.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "nmea.h"
#include "manage.h"
#include "timing.h"
#include "hw/leds.h"
#include "hw/spi.h"
#include "hw/m25_flash.h"

#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

int main(void)
{
  init();

  m25_setup();

  manage_acq_setup();
  cw_setup();
  time_setup();

  led_toggle(LED_RED);

  const double WPR_llh[3] = {D2R*37.038350, D2R*-122.141812, 376.7};

  double WPR_ecef[3];
  wgsllh2ecef(WPR_llh, WPR_ecef);

  channel_measurement_t meas[TRACK_N_CHANNELS];
  navigation_measurement_t nav_meas[TRACK_N_CHANNELS];

  static ephemeris_t es[32];
  while(1)
  {
    for (u32 i = 0; i < 3000; i++)
      __asm__("nop");
    debug_process_messages();
    manage_track();
    manage_acq();

    // Check if there is a new nav msg subframe to process.
    // TODO: move this into a function

    for (u8 i=0; i<TRACK_N_CHANNELS; i++)
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        printf("PRN %d new subframe\n",tracking_channel[i].prn + 1);
        process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
      }

    u8 n_ready = 0;
    for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
      if (es[tracking_channel[i].prn].valid == 1 && \
          es[tracking_channel[i].prn].healthy == 1 && \
          tracking_channel[i].state == TRACKING_RUNNING && \
          tracking_channel[i].TOW_ms > 0) {
        __asm__("CPSID i;");
        tracking_update_measurement(i, &meas[n_ready]);
        __asm__("CPSIE i;");

        n_ready++;
      }
    }
    if (n_ready >= 4) {
      /* Got enough sats/ephemerides, do a solution. */
      calc_navigation_measurement(n_ready, meas, nav_meas, (double)timing_count()/SAMPLE_FREQ, es);

      gnss_solution soln;
      dops_t dops;
      calc_PVT(n_ready, nav_meas, &soln, &dops);

      double mean_range = 0;
      double ranges[n_ready];
      for (u8 i=0; i<n_ready; i++) {
        ranges[i] = predict_range(WPR_ecef, nav_meas[i].tot, &es[meas[i].prn]);
        mean_range += ranges[i];
      }
      mean_range /= n_ready;
      double pr_errs[TRACK_N_CHANNELS];
      double mean = 0;
      for (u8 i=0; i<n_ready; i++) {
        pr_errs[i] = nav_meas[i].pseudorange - (ranges[i] - mean_range + NOMINAL_RANGE);
        mean += pr_errs[i];
      }
      mean /= n_ready;
      for (u8 i=0; i<n_ready; i++) {
        pr_errs[i] -= mean;
      }
      for (u8 i=n_ready; i<TRACK_N_CHANNELS; i++) {
        pr_errs[i] = 0;
      }

      wgsecef2ned_d(soln.pos_ecef, WPR_ecef, soln.pos_ned);
      DO_EVERY_COUNTS(SAMPLE_FREQ/4,
        /*debug_send_msg(MSG_SOLUTION, sizeof(gnss_solution), (u8 *) &soln);*/
        /*debug_send_msg(MSG_DOPS, sizeof(dops_t), (u8 *) &dops);*/

        /*debug_send_msg(MSG_PR_ERRS, sizeof(pr_errs), (u8 *) pr_errs);*/
        nmea_gpgga(&soln, &dops);
      );
      DO_EVERY_COUNTS(SAMPLE_FREQ,
        nmea_gpgsv(n_ready, nav_meas, &soln);
      );
    }

    DO_EVERY_COUNTS(SAMPLE_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_COUNTS(SAMPLE_FREQ/10, // 10 Hz update
      tracking_send_state();
    );

    u32 err = swift_nap_read_error_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}

