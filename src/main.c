/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "cw.h"
#include "sbp.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"
#include "track.h"
#include "acq.h"
#include "nmea.h"
#include "rtcm.h"
#include "manage.h"
#include "timing.h"
#include "position.h"
#include "peripherals/spi.h"
#include "board/leds.h"
#include "board/m25_flash.h"

#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include "settings.h"

channel_measurement_t meas[14];
channel_measurement_t meas_old[14];
navigation_measurement_t nav_meas[14];
navigation_measurement_t nav_meas_old[14];
rtcm_t rtcm;

void sendrtcmnav(ephemeris_t *eph, u8 prn)
{
  memset(&rtcm, 0, sizeof(rtcm));

	rtcm.eph = eph;
  rtcm.prn = prn;
  gen_rtcm3(&rtcm,1019,0);

  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if (settings.ftdi_usart.mode == RTCM)
    usart_write_dma(&ftdi_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uarta_usart.mode == RTCM)
    usart_write_dma(&uarta_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uartb_usart.mode == RTCM)
    usart_write_dma(&uartb_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  __asm__("CPSIE i;");  /* Re-enable interrupts. */
}

void sendrtcmobs(navigation_measurement_t *obs, int nsat, gps_time_t t)
{
  memset(&rtcm, 0, sizeof(rtcm));

	/* observation */
	rtcm.time=t;
	rtcm.n=nsat;
	memcpy(rtcm.obs, obs, sizeof(navigation_measurement_t)*nsat);

	/* GPS observations */
	gen_rtcm3(&rtcm,1002,0);

  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if (settings.ftdi_usart.mode == RTCM)
    usart_write_dma(&ftdi_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uarta_usart.mode == RTCM)
    usart_write_dma(&uarta_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uartb_usart.mode == RTCM)
    usart_write_dma(&uartb_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  __asm__("CPSIE i;");  /* Re-enable interrupts. */
}


int main(void)
{
  init();

  led_toggle(LED_RED);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  u8 nap_git_hash[20];
  nap_conf_rd_git_hash(nap_git_hash);
  printf("SwiftNAP git: ");
  for (u8 i=0; i<20; i++)
    printf("%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    printf(" (unclean)");
  printf("\n");
  printf("SwiftNAP configured with %d tracking channels\n\n", nap_track_n_channels);

  cw_setup();
  manage_acq_setup();
  tick_timer_setup();
  timing_setup();
  position_setup();

  /* TODO: Think about thread safety when updating ephemerides. */
  static ephemeris_t es[32];
  static ephemeris_t es_old[32];
  while(1)
  {
    for (u32 i = 0; i < 3000; i++)
      __asm__("nop");
    sbp_process_messages();
    manage_track();
    manage_acq();

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    memcpy(es_old, es, sizeof(es));
    for (u8 i=0; i<nap_track_n_channels; i++)
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        s8 ret = process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
        if (ret < 0)
          printf("PRN %02d ret %d\n", tracking_channel[i].prn+1, ret);

        if (ret == 1 && !es[tracking_channel[i].prn].healthy)
          printf("PRN %02d unhealthy\n", tracking_channel[i].prn+1);
        if (memcmp(&es[tracking_channel[i].prn], &es_old[tracking_channel[i].prn], sizeof(ephemeris_t))) {
          printf("New ephemeris for PRN %02d\n", tracking_channel[i].prn+1);
          /* TODO: This is a janky way to set the time... */
          gps_time_t t;
          t.wn = es[tracking_channel[i].prn].toe.wn;
          t.tow = tracking_channel[i].TOW_ms / 1000.0;
          if (gpsdifftime(t, es[tracking_channel[i].prn].toe) > 2*24*3600)
            t.wn--;
          else if (gpsdifftime(t, es[tracking_channel[i].prn].toe) < 2*24*3600)
            t.wn++;
          /*set_time(TIME_COARSE, t);*/
        }
        if (es[tracking_channel[i].prn].valid == 1) {
          sendrtcmnav(&es[tracking_channel[i].prn], tracking_channel[i].prn);
        }
      }

    DO_EVERY_TICKS(TICK_FREQ/2,

      u8 n_ready = 0;
      for (u8 i=0; i<nap_track_n_channels; i++) {
        if (es[tracking_channel[i].prn].valid == 1 && \
            es[tracking_channel[i].prn].healthy == 1 && \
            tracking_channel[i].state == TRACKING_RUNNING && \
            tracking_channel[i].TOW_ms > 0) {
          memcpy(meas_old, meas, sizeof(meas));
          __asm__("CPSID i;");
          tracking_update_measurement(i, &meas[n_ready]);
          __asm__("CPSIE i;");

          if (meas[n_ready].snr > 2)
            n_ready++;
        }
      }

      if (n_ready >= 4) {
        /* Got enough sats/ephemerides, do a solution. */
        /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
         * more intelligent with the solution time.
         */
        u64 nav_tc = nap_timing_count();
        memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
        calc_navigation_measurement(n_ready, meas, nav_meas, (double)((u32)nav_tc)/SAMPLE_FREQ, es);

        dops_t dops;
        if (calc_PVT(n_ready, nav_meas, &position_solution, &dops) == 0) {
          position_updated();

          sbp_send_msg(MSG_SOLUTION, sizeof(gnss_solution), (u8 *) &position_solution);
          nmea_gpgga(&position_solution, &dops);

          sendrtcmobs(nav_meas, n_ready, position_solution.time);

          DO_EVERY(10,
            sbp_send_msg(MSG_DOPS, sizeof(dops_t), (u8 *) &dops);
            nmea_gpgsv(n_ready, nav_meas, &position_solution);
          );
        }
      }
    );

    DO_EVERY_TICKS(TICK_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_TICKS(TICK_FREQ/10, // 10 Hz update
      tracking_send_state();
    );

    u32 err = nap_error_rd_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}

