/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ch.h"

#include <libopencm3/stm32/f4/timer.h>

#include <libswiftnav/pvt.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/constants.h>

#include "board/leds.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"
#include "sbp.h"
#include "init.h"
#include "manage.h"
#include "track.h"
#include "timing.h"
#include "position.h"
#include "system_monitor.h"

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 130944000
#endif

#define MAX_SATS 14

/* TODO: Think about thread safety when updating ephemerides. */
ephemeris_t es[32];
ephemeris_t es_old[32];

channel_measurement_t meas[MAX_SATS];
navigation_measurement_t nav_meas[MAX_SATS];
navigation_measurement_t nav_meas_old[MAX_SATS];
u8 n_obs;

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

static WORKING_AREA(wa_sbp_thread, 4096);
static msg_t sbp_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    led_toggle(LED_GREEN);
    chThdSleepMilliseconds(50);
    sbp_process_messages();
  }

  return 0;
}

static WORKING_AREA(wa_nav_msg_thread, 4096);
static msg_t nav_msg_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(1000);

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    /* TODO: This should be trigged by a semaphore from the tracking loop, not
     * just ran periodically. */

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
        /*if (es[tracking_channel[i].prn].valid == 1) {*/
          /*sendrtcmnav(&es[tracking_channel[i].prn], tracking_channel[i].prn);*/
        /*}*/
      }
  }

  return 0;
}

static Thread *tp = NULL;
#define tim5_isr Vector108
#define NVIC_TIM5_IRQ 50
void tim5_isr()
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  /* Wake up processing thread */
  if (tp != NULL) {
    chSchReadyI(tp);
    tp = NULL;
  }

  timer_clear_flag(TIM5, TIM_SR_UIF);

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

static WORKING_AREA(wa_solution_thread, 8000);
static msg_t solution_thread(void *arg)
{
  (void)arg;

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chSysLock();
    tp = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    chSysUnlock();

    led_toggle(LED_RED);

    u8 n_ready = 0;
    for (u8 i=0; i<nap_track_n_channels; i++) {
      if (tracking_channel[i].state == TRACKING_RUNNING && \
          es[tracking_channel[i].prn].valid == 1 && \
          es[tracking_channel[i].prn].healthy == 1 && \
          tracking_channel[i].TOW_ms > 0) {
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
      static u8 n_ready_old = 0;
      u64 nav_tc = nap_timing_count();
      calc_navigation_measurement(n_ready, meas, nav_meas, (double)((u32)nav_tc)/SAMPLE_FREQ, es);

      navigation_measurement_t nav_meas_tdcp[MAX_SATS];
      u8 n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp);

      dops_t dops;
      if (calc_PVT(n_ready_tdcp, nav_meas_tdcp, &position_solution, &dops) == 0) {
        position_updated();

#define SOLN_FREQ 2.0

        double expected_tow = round(position_solution.time.tow*SOLN_FREQ) / SOLN_FREQ;
        double t_err = expected_tow - position_solution.time.tow;

        for (u8 i=0; i<n_ready_tdcp; i++) {
          nav_meas_tdcp[i].pseudorange -= t_err * nav_meas_tdcp[i].doppler * (GPS_C / GPS_L1_HZ);
          nav_meas_tdcp[i].carrier_phase += t_err * nav_meas_tdcp[i].doppler;
          if (fabs(t_err) > 0.01)
            printf("dphase[%d] = %f * %f = %f\n", i, t_err, nav_meas_tdcp[i].doppler, t_err * nav_meas_tdcp[i].doppler);
        }
        gps_time_t new_obs_time;
        new_obs_time.wn = position_solution.time.wn;
        new_obs_time.tow = expected_tow;

        n_obs = n_ready_tdcp;

        static u8 obs_count = 0;
        msg_obs_hdr_t obs_hdr = { .t = new_obs_time, .count = obs_count, .n_obs = n_ready_tdcp };
        sbp_send_msg(MSG_OBS_HDR, sizeof(obs_hdr), (u8 *)&obs_hdr);
        send_observations(n_ready_tdcp, nav_meas_tdcp);
        obs_count++;

        double dt = expected_tow + (1/SOLN_FREQ) - position_solution.time.tow;

        /* Limit dt to 2 seconds maximum to prevent hang if dt calculated incorrectly. */
        if (dt > 2)
          dt = 2;

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
        pos_llh.n_sats = n_ready_tdcp;
        pos_llh.flags = 0;
        sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

        sbp_vel_ned_t vel_ned;
        vel_ned.tow = round(position_solution.time.tow * 1e3);
        vel_ned.n = round(position_solution.vel_ned[0] * 1e3);
        vel_ned.e = round(position_solution.vel_ned[1] * 1e3);
        vel_ned.d = round(position_solution.vel_ned[2] * 1e3);
        vel_ned.h_accuracy = 0;
        vel_ned.v_accuracy = 0;
        vel_ned.n_sats = n_ready_tdcp;
        vel_ned.flags = 0;
        sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

        /*nmea_gpgga(&position_solution, &dops);*/

        /*sendrtcmobs(nav_meas_tdcp, n_ready_tdcp, position_solution.time);*/

        DO_EVERY(10,
          sbp_dops_t sbp_dops;
          sbp_dops.pdop = round(dops.pdop * 100);
          sbp_dops.gdop = round(dops.gdop * 100);
          sbp_dops.tdop = round(dops.tdop * 100);
          sbp_dops.hdop = round(dops.hdop * 100);
          sbp_dops.vdop = round(dops.vdop * 100);
          sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
          /*nmea_gpgsv(n_ready_tdcp, nav_meas_tdcp, &position_solution);*/
        );
      }
      memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
      n_ready_old = n_ready;
    }

  }
  return 0;
}

void soln_timer_setup()
{
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvicEnableVector(NVIC_TIM5_IRQ, CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+1));
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
  /* Initialise SysTick timer that will be used as the ChibiOS kernel tick
   * timer. */
  STBase->RVR = SYSTEM_CLOCK / CH_FREQUENCY - 1;
  STBase->CVR = 0;
  STBase->CSR = CLKSOURCE_CORE_BITS | ENABLE_ON_BITS | TICKINT_ENABLED_BITS;

  /* Kernel initialization, the main() function becomes a thread and the RTOS
   * is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  init(1);

  printf("\n\nFirmware info - git: " GIT_VERSION \
         ", built: " __DATE__ " " __TIME__ "\n");
  u8 nap_git_hash[20];
  nap_conf_rd_git_hash(nap_git_hash);
  printf("SwiftNAP git: ");
  for (u8 i=0; i<20; i++)
    printf("%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    printf(" (unclean)");
  printf("\n");
  printf("SwiftNAP configured with %d tracking channels\n\n",
         nap_track_n_channels);

  timing_setup();
  position_setup();

  manage_acq_setup();
  manage_track_setup();
  system_monitor_setup();
  soln_timer_setup();

  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread), NORMALPRIO-1, nav_msg_thread, NULL);
  chThdCreateStatic(wa_sbp_thread, sizeof(wa_sbp_thread), HIGHPRIO-22, sbp_thread, NULL);
  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread), HIGHPRIO-1, solution_thread, NULL);

  while (1) {
    chThdSleepSeconds(60);
  }
}
