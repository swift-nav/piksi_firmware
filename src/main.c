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

#include <stdlib.h>
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
#include <libswiftnav/sbp.h>
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>

#include "settings.h"

#define MAX_SATS 14

channel_measurement_t meas[MAX_SATS];
channel_measurement_t meas_old[MAX_SATS];
navigation_measurement_t nav_meas[MAX_SATS];
navigation_measurement_t nav_meas_old[MAX_SATS];
u8 n_obs;

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

int _getpid()
{
  return 1;
}
void _exit(int rc)
{
  (void)rc;
  while(1);
}
int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  return -1; /* Always fails */
}

void send_observations(u8 n, navigation_measurement_t *m)
{
  msg_obs_t obs;
  for (u8 i=0; i<n; i++) {
    obs.prn = m[i].prn;
    obs.P = m[i].raw_pseudorange;
    obs.L = m[i].carrier_phase;
    obs.D = m[i].doppler;
    obs.snr = m[i].snr;
    obs.lock_count = 255;
    obs.flags = 0;
    obs.obs_n = i;
    sbp_send_msg(MSG_OBS, sizeof(obs), (u8 *)&obs);
  }
}
  /* TODO: Think about thread safety when updating ephemerides. */
  ephemeris_t es[32];
  ephemeris_t es_old[32];

void tim5_isr()
{
  led_toggle(LED_RED);

    /*static u32 last_tow = 0;*/
    if (tracking_channel[0].state == TRACKING_RUNNING)
    {
      /*last_tow = tracking_channel[0].TOW_ms;*/

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

          n_obs = n_ready;

          static u8 obs_count = 0;
          msg_obs_hdr_t obs_hdr = { .t = position_solution.time, .count = obs_count, .n_obs = n_ready };
          sbp_send_msg(MSG_OBS_HDR, sizeof(obs_hdr), (u8 *)&obs_hdr);
          send_observations(n_ready, nav_meas);
          obs_count++;

          double next_sec = ceil(position_solution.time.tow);
          double dt = next_sec - position_solution.time.tow;
          if (dt < 0.1)
            dt += 1;

          timer_set_period(TIM5, round(65472000 * dt));


          sbp_gps_time_t gps_time;
          gps_time.wn = position_solution.time.wn;
          gps_time.tow = round(position_solution.time.tow * 1e3);
          gps_time.ns = round((position_solution.time.tow - gps_time.tow*1e-3) * 1e9);
          gps_time.flags = 0;
          sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

          sbp_pos_llh_t pos_llh;
          pos_llh.tow = round(position_solution.time.tow * 1e3);
          pos_llh.lat = position_solution.pos_llh[0] * R2D;
          pos_llh.lon = position_solution.pos_llh[1] * R2D;
          pos_llh.height = position_solution.pos_llh[2];
          pos_llh.h_accuracy = 0;
          pos_llh.v_accuracy = 0;
          pos_llh.n_sats = n_ready;
          pos_llh.flags = 0;
          sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

          sbp_vel_ned_t vel_ned;
          vel_ned.tow = round(position_solution.time.tow * 1e3);
          vel_ned.n = round(position_solution.vel_ned[0] * 1e3);
          vel_ned.e = round(position_solution.vel_ned[1] * 1e3);
          vel_ned.d = round(position_solution.vel_ned[2] * 1e3);
          vel_ned.h_accuracy = 0;
          vel_ned.v_accuracy = 0;
          vel_ned.n_sats = n_ready;
          vel_ned.flags = 0;
          sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

          nmea_gpgga(&position_solution, &dops);

          sendrtcmobs(nav_meas, n_ready, position_solution.time);

          DO_EVERY(10,
            sbp_dops_t sbp_dops;
            sbp_dops.pdop = round(dops.pdop * 100);
            sbp_dops.gdop = round(dops.gdop * 100);
            sbp_dops.tdop = round(dops.tdop * 100);
            sbp_dops.hdop = round(dops.hdop * 100);
            sbp_dops.vdop = round(dops.vdop * 100);
            sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
            nmea_gpgsv(n_ready, nav_meas, &position_solution);
          );
        }
      }
    }


  timer_clear_flag(TIM5, TIM_SR_UIF);
}

void timer_setup()
{
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvic_enable_irq(NVIC_TIM5_IRQ);
  nvic_set_priority(NVIC_TIM5_IRQ, 22);
  nvic_set_priority(NVIC_EXTI1_IRQ, 1);
  timer_reset(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, 0);
  timer_disable_preload(TIM5);
  timer_set_period(TIM5, 65472000); /* 1 second. */
  timer_enable_counter(TIM5);
  timer_enable_irq(TIM5, TIM_DIER_UIE);
}


int main(void)
{
  init(1);


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
  timer_setup();

  while(1)
  {
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


    DO_EVERY_TICKS(TICK_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_TICKS(TICK_FREQ/5, // 5 Hz update
      tracking_send_state();
    );

    u32 err = nap_error_rd_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}





