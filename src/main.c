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
#include "nap/nap_common.h"
#include "nap/track_channel.h"
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

  manage_acq_setup();
  cw_setup();
  time_setup();

  led_toggle(LED_RED);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("FPGA configured with %d tracking channels\n", TRACK_N_CHANNELS);

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
        process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
      }

    /*u32 foo;*/

    DO_EVERY_COUNTS(TICK_FREQ/5,
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
        /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
         * more intelligent with the solution time.
         */

        /*printf("n_ready = %d\n", n_ready);*/
        /*foo = time_ticks();*/

        calc_navigation_measurement(n_ready, meas, nav_meas, (double)((u32)nap_timing_count())/SAMPLE_FREQ, es);

        /*printf("Nav meas took: %.2fms\n", 1e3*(double)(time_ticks() - foo) / TICK_FREQ);*/
        /*foo = time_ticks();*/

        gnss_solution soln;
        dops_t dops;
        if (calc_PVT(n_ready, nav_meas, &soln, &dops) == 0) {
          /*printf("calc_PVT took: %.2fms\n", 1e3*(double)(time_ticks() - foo) / TICK_FREQ);*/
          /*foo = time_ticks();*/

          wgsecef2ned_d(soln.pos_ecef, WPR_ecef, soln.pos_ned);

          debug_send_msg(MSG_SOLUTION, sizeof(gnss_solution), (u8 *) &soln);
          /*nmea_gpgga(&soln, &dops);*/

          /*DO_EVERY(1,*/
            /*debug_send_msg(MSG_DOPS, sizeof(dops_t), (u8 *) &dops);*/
            /*nmea_gpgsv(n_ready, nav_meas, &soln);*/
          /*);*/
        }
      }
    );

    /* USART TX testing */
    gnss_solution soln;
    dops_t dops;
//    DO_EVERY(5,
    debug_send_msg(MSG_DOPS, sizeof(dops_t), (u8 *) &dops);
    debug_send_msg(MSG_SOLUTION, sizeof(gnss_solution), (u8 *) &soln);
//    );

    DO_EVERY_COUNTS(TICK_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_COUNTS(TICK_FREQ/10, // 10 Hz update
      tracking_send_state();
    );

    u32 err = nap_read_error_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}

