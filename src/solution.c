/*
 * Copyright (C) 2014 Swift Navigation Inc.
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
#include <string.h>
#include <stdlib.h>

#include <libswiftnav/sbp_utils.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>

#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/rcc.h>

#include <ch.h>

#include "board/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "solution.h"
#include "manage.h"
#include "simulator.h"

void solution_send_sbp(gnss_solution *soln, dops_t *dops)
{
  /* Send GPS_TIME message first. */
  sbp_gps_time_t gps_time;
  sbp_make_gps_time(&gps_time, &soln->time, 0);
  sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

  /* Position in LLH. */
  sbp_pos_llh_t pos_llh;
  sbp_make_pos_llh(&pos_llh, soln, 0);
  sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

  /* Velocity in NED. */
  sbp_vel_ned_t vel_ned;
  sbp_make_vel_ned(&vel_ned, soln, 0);
  sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

  DO_EVERY(10,
    sbp_dops_t sbp_dops;
    sbp_make_dops(&sbp_dops, dops);
    sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
  );
}

void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm)
{
  nmea_gpgga(soln, dops);

  DO_EVERY(10,
    nmea_gpgsv(n, nm, soln);
  );
}

void solution_send_baseline(gps_time_t *t, u8 n_sats, double b_ecef[3],
                            double ref_ecef[3])
{
  if (1) {
    sbp_baseline_ecef_t sbp_ecef = {
      .tow = t->tow,
      .x = (s32)round(1e3 * b_ecef[0]),
      .y = (s32)round(1e3 * b_ecef[1]),
      .z = (s32)round(1e3 * b_ecef[2]),
      .n_sats = n_sats,
    };
    sbp_send_msg(SBP_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);
  }

  if (1) {
    double b_ned[3];
    wgsecef2ned(b_ecef, ref_ecef, b_ned);

    sbp_baseline_ned_t sbp_ned = {
      .tow = t->tow,
      .n = (s32)round(1e3 * b_ned[0]),
      .e = (s32)round(1e3 * b_ned[1]),
      .d = (s32)round(1e3 * b_ned[2]),
      .n_sats = n_sats,
    };
    sbp_send_msg(SBP_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);
  }
}

#define MAX_SATS 14
#define MAX_CHANNELS 14

extern ephemeris_t es[32];
channel_measurement_t meas[MAX_SATS];
navigation_measurement_t nav_meas[MAX_SATS];
navigation_measurement_t nav_meas_old[MAX_SATS];

navigation_measurement_t nav_meas_base[MAX_CHANNELS];
u8 n_base;
double tow_base = -1;

void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  tow_base = ((gps_time_t *)msg)->tow;
  n_base = (len - sizeof(gps_time_t)) / sizeof(msg_obs_t);
  msg_obs_t *obs = (msg_obs_t *)(msg + sizeof(gps_time_t));
  for (u8 i=0; i<n_base; i++) {
    nav_meas_base[i].prn = obs[i].prn;
    nav_meas_base[i].raw_pseudorange = obs[i].P;
    nav_meas_base[i].carrier_phase = obs[i].L;
    nav_meas_base[i].snr = obs[i].snr;
  }

  /* Ensure observations sorted by PRN. */
  qsort(nav_meas_base, n_base, sizeof(navigation_measurement_t), nav_meas_cmp);

  static u32 obs_count = 0;
  obs_count++;
  if (obs_count % 20 == 0) {
    printf("Obs count: %u\n", (unsigned int)obs_count);
  }
}

void send_observations(u8 n, gps_time_t *t, navigation_measurement_t *m)
{
  static u8 buff[256];

  memcpy(buff, t, sizeof(gps_time_t));
  msg_obs_t *obs = (msg_obs_t *)&buff[sizeof(gps_time_t)];
  if (n * sizeof(msg_obs_t) > 255 - sizeof(gps_time_t))
    n = 255 / sizeof(msg_obs_t);
  for (u8 i=0; i<n; i++) {
    obs[i].prn = m[i].prn;
    obs[i].P = m[i].raw_pseudorange;
    obs[i].L = m[i].carrier_phase;
    obs[i].snr = m[i].snr;
  }
  sbp_send_msg(MSG_NEW_OBS, sizeof(gps_time_t) + n*sizeof(msg_obs_t), buff);
}

/*
void send_observations(u8 n, gps_time_t *t, navigation_measurement_t *m)
{
  static u8 obs_count = 0;
  msg_obs_hdr_t obs_hdr = {
    .t = *t,
    .count = obs_count,
    .n_obs = n
  };
  obs_count++;
  sbp_send_msg(MSG_OBS_HDR, sizeof(obs_hdr), (u8 *)&obs_hdr);

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
*/

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
  chRegSetThreadName("solution");

  while (TRUE) {
    /* Waiting for the timer IRQ fire.*/
    chSysLock();
    tp = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    chSysUnlock();

    if (simulation_enabled()) {
      led_on(LED_RED);
    } else {
      led_toggle(LED_RED);      
    }

    u8 n_ready = 0;
    for (u8 i=0; i<nap_track_n_channels; i++) {
      if (use_tracking_channel(i)) {
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
      calc_navigation_measurement(n_ready, meas, nav_meas,
                                  (double)((u32)nav_tc)/SAMPLE_FREQ, es);

      navigation_measurement_t nav_meas_tdcp[MAX_SATS];
      u8 n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old,
                                     nav_meas_old, nav_meas_tdcp);

      dops_t dops;
      if (calc_PVT(n_ready_tdcp, nav_meas_tdcp, &position_solution, &dops) == 0) {
        position_updated();

#define SOLN_FREQ 2.0

        double expected_tow = round(position_solution.time.tow*SOLN_FREQ)
                                / SOLN_FREQ;
        double t_err = expected_tow - position_solution.time.tow;

        for (u8 i=0; i<n_ready_tdcp; i++) {
          nav_meas_tdcp[i].pseudorange -= t_err * nav_meas_tdcp[i].doppler *
            (GPS_C / GPS_L1_HZ);
          nav_meas_tdcp[i].carrier_phase += t_err * nav_meas_tdcp[i].doppler;
        }

        /* Only send observations that are closely aligned with the desired
         * solution epochs to ensure they haven't been propagated too far. */
        if (fabs(t_err) < 10e-3) {
          gps_time_t new_obs_time;
          new_obs_time.wn = position_solution.time.wn;
          new_obs_time.tow = expected_tow;
          /* Output obervations. */
          send_observations(n_ready_tdcp, &new_obs_time, nav_meas_tdcp);
        }

        if (!simulation_enabled()) {
          /* Output solution. */
          solution_send_sbp(&position_solution, &dops);
          solution_send_nmea(&position_solution, &dops, n_ready_tdcp, nav_meas_tdcp);
        }

        /* Calculate time till the next desired solution epoch. */
        double dt = expected_tow + (1/SOLN_FREQ) - position_solution.time.tow;

        /* Limit dt to 2 seconds maximum to prevent hang if dt calculated
         * incorrectly. */
        if (dt > 2)
          dt = 2;

        /* Reset timer period with the count that we will estimate will being
         * us up to the next solution time. */
        timer_set_period(TIM5, round(65472000 * dt));
      }

      /* Store current observations for next time for
       * TDCP Doppler caluclation. */
      memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
      n_ready_old = n_ready;
    }

    //Here we do all the nice simulation-related stuff.
    if (simulation_enabled_for(SIM_PVT)) {

      //Set the timer period appropriately
      timer_set_period(TIM5, round(65472000 * (1.0/SOLN_FREQ)));

      simulation_step();

      //Then we send fake messages
      solution_send_sbp(simulation_current_gnss_solution(), simulation_current_dops_solution());

    }
  }
  return 0;
}

void solution_setup()
{
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvicEnableVector(NVIC_TIM5_IRQ,
      CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+1));
  timer_reset(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, 0);
  timer_disable_preload(TIM5);
  timer_set_period(TIM5, 65472000); /* 1 second. */
  timer_enable_counter(TIM5);
  timer_enable_irq(TIM5, TIM_DIER_UIE);

  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread),
                    HIGHPRIO-1, solution_thread, NULL);
}

