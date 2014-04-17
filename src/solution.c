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
#include <libswiftnav/coord_system.h>
#include <libswiftnav/single_diff.h>
#include <libswiftnav/dgnss_management.h>
#include <libswiftnav/ambiguity_test.h>
#include <libswiftnav/stupid_filter.h>

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
#include "settings.h"

Mutex base_obs_lock;
BinarySemaphore base_obs_received;
MemoryPool obs_buff_pool;
Mailbox obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_TIME_MATCHED;
dgnss_filter_t dgnss_filter = FILTER_FIXED;

double known_baseline[3] = {0, 0, 0};

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
                            double ref_ecef[3], u8 flags)
{
  if (1) {
    sbp_baseline_ecef_t sbp_ecef = {
      .tow = t->tow,
      .x = (s32)round(1e3 * b_ecef[0]),
      .y = (s32)round(1e3 * b_ecef[1]),
      .z = (s32)round(1e3 * b_ecef[2]),
      .accuracy = 0,
      .n_sats = n_sats,
      .flags = flags
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
      .h_accuracy = 0,
      .v_accuracy = 0,
      .n_sats = n_sats,
      .flags = flags
    };
    sbp_send_msg(SBP_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);
  }
}

extern ephemeris_t es[MAX_SATS];

obss_t base_obss;

void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  /* Sender ID of zero means that the messages are relayed observations,
   * ignore them. */
  if (sender_id == 0)
    return;

  /* Relay observations using sender_if = 0. */
  sbp_send_msg_(MSG_NEW_OBS, len, msg, 0);

  gps_time_t *t = (gps_time_t *)msg;
  double epoch_count = t->tow * SOLN_FREQ;

  if (fabs(epoch_count - round(epoch_count)) > TIME_MATCH_THRESHOLD) {
    printf("Unaligned observation from base station ignored.\n");
    return;
  }

  /* Lock mutex before modifying base_obss. */
  chMtxLock(&base_obs_lock);

  base_obss.t = *t;
  base_obss.n = (len - sizeof(gps_time_t)) / sizeof(msg_obs_t);
  msg_obs_t *obs = (msg_obs_t *)(msg + sizeof(gps_time_t));
  for (u8 i=0; i<base_obss.n; i++) {
    base_obss.nm[i].prn = obs[i].prn;
    base_obss.nm[i].raw_pseudorange = obs[i].P;
    base_obss.nm[i].carrier_phase = obs[i].L;
    base_obss.nm[i].snr = obs[i].snr;
  }

  /* Ensure observations sorted by PRN. */
  qsort(base_obss.nm, base_obss.n,
        sizeof(navigation_measurement_t), nav_meas_cmp);

  /* Unlock mutex. */
  chMtxUnlock();

  /* Signal that a base observation has been received. */
  chBSemSignal(&base_obs_received);
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

static WORKING_AREA_CCM(wa_solution_thread, 5000);
static msg_t solution_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("solution");

  static navigation_measurement_t nav_meas_old[MAX_CHANNELS];

  while (TRUE) {
    /* Waiting for the timer IRQ fire.*/
    chSysLock();
    tp = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    chSysUnlock();

    u8 n_ready = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    for (u8 i=0; i<nap_track_n_channels; i++) {
      if (use_tracking_channel(i)) {
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
      static u8 n_ready_old = 0;
      u64 nav_tc = nap_timing_count();
      static navigation_measurement_t nav_meas[MAX_CHANNELS];
      calc_navigation_measurement(n_ready, meas, nav_meas,
                                  (double)((u32)nav_tc)/SAMPLE_FREQ, es);

      static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];
      u8 n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old,
                                     nav_meas_old, nav_meas_tdcp);

      /* Store current observations for next time for
       * TDCP Doppler calculation. */
      memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
      n_ready_old = n_ready;

      dops_t dops;
      if (calc_PVT(n_ready_tdcp, nav_meas_tdcp, &position_solution, &dops) == 0) {

        /* Update global position solution state. */
        position_updated();

        if (!simulation_enabled()) {
          /* Output solution. */
          solution_send_sbp(&position_solution, &dops);
          solution_send_nmea(&position_solution, &dops,
                             n_ready_tdcp, nav_meas_tdcp);
        }

        /* If we have a recent set of observations from the base station, do a
         * differential solution. */
        double pdt;
        chMtxLock(&base_obs_lock);
        if (base_obss.n > 0) {
          if ((pdt = gpsdifftime(position_solution.time, base_obss.t))
                < MAX_AGE_OF_DIFFERENTIAL) {

            /* Propagate base station observations to the current time and
             * process a low-latency differential solution. */

            /* Hook in low-latency filter here. */
            if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
              /*solution_send_baseline(t, n_sds, b, position_solution.pos_ecef);*/
            }

          }
        }
        chMtxUnlock();

        /* Calculate the time of the nearest solution epoch, were we expected
         * to be and calculate how far we were away from it. */
        double expected_tow = round(position_solution.time.tow*SOLN_FREQ)
                                / SOLN_FREQ;
        double t_err = expected_tow - position_solution.time.tow;

        /* Only send observations that are closely aligned with the desired
         * solution epochs to ensure they haven't been propagated too far. */
        if (fabs(t_err) < OBS_PROPAGATION_LIMIT) {
          /* Propagate observation to desired time. */
          for (u8 i=0; i<n_ready_tdcp; i++) {
            nav_meas_tdcp[i].pseudorange -= t_err * nav_meas_tdcp[i].doppler *
              (GPS_C / GPS_L1_HZ);
            nav_meas_tdcp[i].carrier_phase += t_err * nav_meas_tdcp[i].doppler;
          }

          /* Update observation time. */
          gps_time_t new_obs_time;
          new_obs_time.wn = position_solution.time.wn;
          new_obs_time.tow = expected_tow;

          if (!simulation_enabled()) {
            /* Output obervations. */
            send_observations(n_ready_tdcp, &new_obs_time, nav_meas_tdcp);
          }

          /* TODO: use a buffer from the pool from the start instead of
           * allocating nav_meas_tdcp as well. Downside, if we don't end up
           * pushing the message into the mailbox then we just wasted an
           * observation from the mailbox for no good reason. */

          obss_t *obs = chPoolAlloc(&obs_buff_pool);
          msg_t ret;
          if (obs == NULL) {
            /* Pool is empty, grab a buffer from the mailbox instead, i.e.
             * overwrite the oldest item in the queue. */
            ret = chMBFetch(&obs_mailbox, (msg_t *)&obs, TIME_IMMEDIATE);
            if (ret != RDY_OK) {
              printf("ERROR: Pool full and mailbox empty!\n");
            }
          }
          obs->t = new_obs_time;
          obs->n = n_ready_tdcp;
          memcpy(obs->nm, nav_meas_tdcp, obs->n * sizeof(navigation_measurement_t));
          ret = chMBPost(&obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
          if (ret != RDY_OK) {
            /* We could grab another item from the mailbox, discard it and then
             * post our obs again but if the size of the mailbox and the pool
             * are equal then we should have already handled the case where the
             * mailbox is full when we handled the case that the pool was full.
             * */
            printf("ERROR: Mailbox should have space!\n");
          }
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

    }

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {

      /* Set the timer period appropriately. */
      timer_set_period(TIM5, round(65472000 * (1.0/SOLN_FREQ)));

      simulation_step();

      if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
        /* Then we send fake messages. */
        solution_send_sbp(simulation_current_gnss_solution(),
                          simulation_current_dops_solution());
      }

      if (simulation_enabled_for(SIMULATION_MODE_RTK)) {
        solution_send_baseline(&simulation_current_gnss_solution()->time,
          simulation_current_num_sats(),
          simulation_current_baseline_ecef(),
          simulation_ref_ecef(), 0);

        send_observations(simulation_current_num_sats(),
          &simulation_current_gnss_solution()->time,
          simulation_current_navigation_measurements());
      }
    }
  }
  return 0;
}

static bool init_done = false;
static bool init_known_base = false;
static bool reset_iar = false;

void process_matched_obs(u8 n_sds, gps_time_t *t, sdiff_t *sds, double dt)
{
  (void)n_sds; (void)sds;

  /* Hook Float KF and AR filters in here. */
  if (n_sds > 4) {
    if (init_known_base) {
      /* Calculate ambiguities from known baseline. */
      printf("Initializing using known baseline\n");
      double known_baseline_ecef[3];
      wgsned2ecef(known_baseline, position_solution.pos_ecef,
                  known_baseline_ecef);
      dgnss_init_known_baseline(n_sds, sds, position_solution.pos_ecef,
                                known_baseline_ecef);
      init_known_base = false;
    }
    if (!init_done) {
      /* Initialize filters. */
      printf("Initializing DGNSS filters\n");
      dgnss_init(n_sds, sds, position_solution.pos_ecef, dt);
      init_done = 1;
    } else {
      if (reset_iar) {
        dgnss_reset_iar();
        reset_iar = false;
      }
      /* Update filters. */
      dgnss_update(n_sds, sds, position_solution.pos_ecef, dt);
      /* If we are in time matched mode then calculate and output the baseline
       * for this observation. */
      if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
        double b[3];
        u8 num_used;
        switch (dgnss_filter) {
        case FILTER_FIXED:
          /* Calculate least squares solution using ambiguities from IAR. */
          dgnss_fixed_baseline(n_sds, sds, position_solution.pos_ecef,
                               &num_used, b);
          msg_iar_state_t iar_state = { .num_hyps = dgnss_iar_num_hyps() };
          sbp_send_msg(MSG_IAR_STATE, sizeof(msg_iar_state_t), (u8 *)&iar_state);
          u8 flags = (dgnss_iar_resolved()) ? 1 : 0;
          solution_send_baseline(t, num_used, b, position_solution.pos_ecef, flags);
          break;
        case FILTER_FLOAT:
          dgnss_new_float_baseline(n_sds, sds,
                                   position_solution.pos_ecef, &num_used, b);
          solution_send_baseline(t, num_used, b, position_solution.pos_ecef, 0);
          break;
        case FILTER_OLD_FLOAT:
          dgnss_float_baseline(&num_used, b);
          solution_send_baseline(t, num_used, b, position_solution.pos_ecef, 0);
          break;
        }
      }
    }
  }
}

static WORKING_AREA_CCM(wa_time_matched_obs_thread, 10000);
static msg_t time_matched_obs_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("time matched obs");
  while (1) {
    /* Wait for a new observation to arrive from the base station. */
    chBSemWait(&base_obs_received);

    /* Blink red LED for 20ms. */
    systime_t t_blink = chTimeNow() + MS2ST(50);
    led_on(LED_RED);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&obs_mailbox, (msg_t *)&obss, TIME_IMMEDIATE)
            == RDY_OK) {
      chMtxLock(&base_obs_lock);

      double dt = gpsdifftime(obss->t, base_obss.t);

      if (fabs(dt) < TIME_MATCH_THRESHOLD) {
        /* Times match! Process obs and base_obss */
        static sdiff_t sds[MAX_CHANNELS];
        u8 n_sds = single_diff(
            obss->n, obss->nm,
            base_obss.n, base_obss.nm,
            sds
        );
        process_matched_obs(n_sds, &obss->t, sds, 1.0 / SOLN_FREQ);
        chPoolFree(&obs_buff_pool, obss);
        chMtxUnlock();
        break;
      } else if (dt > 0) {
        /* Time of base obs before time of local obs, we must not have a local
         * observation matching this base observation, break and wait for a new
         * base observation. */

        /* In practice this should basically never happen so lets make a note
         * if it does. */
        printf("Obs Matching: t_base < t_rover (%f)\n", dt);

        /* Return the buffer to the mailbox so we can try it again later. */
        msg_t ret = chMBPost(&obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
        if (ret != RDY_OK) {
          /* Something went wrong with returning it to the buffer, better just
           * free it and carry on. */
          printf("Obs Matching: mailbox full, discarding observation!\n");
          chPoolFree(&obs_buff_pool, obss);
        }
        chMtxUnlock();
        break;
      } else {
        /* Time of base obs later than time of local obs,
         * keep moving through the mailbox. */
        chPoolFree(&obs_buff_pool, obss);
        chMtxUnlock();
      }
    }

    chSysLock();
    if (t_blink > chTimeNow()) {
      chThdSleepS(t_blink - chTimeNow());
    }
    chSysUnlock();

    led_off(LED_RED);
  }
  return 0;
}

void reset_filters_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)context;
  switch (msg[0]) {
  case 0:
    printf("Filter reset requested\n");
    init_done = false;
    break;
  case 1:
    printf("IAR reset requested\n");
    reset_iar = true;
    break;
  default:
    break;
  }
}

void init_base_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void)context;
  init_known_base = true;
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

  static const char const *dgnss_soln_mode_enum[] = {
    "Low Latency",
    "Time Matched",
    NULL
  };
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(dgnss_soln_mode_enum,
                                                        &dgnss_soln_mode_setting);
  SETTING("solution", "dgnss_solution_mode",
          dgnss_soln_mode, TYPE_GNSS_SOLN_MODE);

  static const char const *dgnss_filter_enum[] = {
    "Float",
    "Old Float",
    "Fixed",
    NULL
  };
  static struct setting_type dgnss_filter_setting;
  int TYPE_GNSS_FILTER = settings_type_register_enum(dgnss_filter_enum,
                                                     &dgnss_filter_setting);
  SETTING("solution", "dgnss_filter",
          dgnss_filter, TYPE_GNSS_FILTER);

  SETTING("solution", "known_baseline_n", known_baseline[0], TYPE_FLOAT);
  SETTING("solution", "known_baseline_e", known_baseline[1], TYPE_FLOAT);
  SETTING("solution", "known_baseline_d", known_baseline[2], TYPE_FLOAT);

  SETTING("iar", "phase_var", dgnss_settings.phase_var_test, TYPE_FLOAT);
  SETTING("iar", "code_var", dgnss_settings.code_var_test, TYPE_FLOAT);

  SETTING("float_kf", "phase_var", dgnss_settings.phase_var_kf, TYPE_FLOAT);
  SETTING("float_kf", "code_var", dgnss_settings.code_var_kf, TYPE_FLOAT);
  SETTING("float_kf", "amb_init_var", dgnss_settings.amb_init_var, TYPE_FLOAT);
  SETTING("float_kf", "new_amb_var", dgnss_settings.new_int_var, TYPE_FLOAT);

  SETTING("old_kf", "pos_trans_var", dgnss_settings.pos_trans_var, TYPE_FLOAT);
  SETTING("old_kf", "vel_trans_var", dgnss_settings.vel_trans_var, TYPE_FLOAT);
  SETTING("old_kf", "int_trans_var", dgnss_settings.int_trans_var, TYPE_FLOAT);
  SETTING("old_kf", "pos_init_var", dgnss_settings.pos_init_var, TYPE_FLOAT);
  SETTING("old_kf", "vel_init_var", dgnss_settings.vel_init_var, TYPE_FLOAT);

  chMtxInit(&base_obs_lock);
  chBSemInit(&base_obs_received, TRUE);
  static msg_t obs_mailbox_buff[OBS_N_BUFF];
  chMBInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);
  chPoolInit(&obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[OBS_N_BUFF] _CCM;
  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread),
                    HIGHPRIO-1, solution_thread, NULL);

  chThdCreateStatic(wa_time_matched_obs_thread, sizeof(wa_time_matched_obs_thread),
                    LOWPRIO, time_matched_obs_thread, NULL);

  static sbp_msg_callbacks_node_t obs_node;
  sbp_register_cbk(
    MSG_NEW_OBS,
    &obs_callback,
    &obs_node
  );

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
    MSG_RESET_FILTERS,
    &reset_filters_callback,
    &reset_filters_node
  );

  static sbp_msg_callbacks_node_t init_base_node;
  sbp_register_cbk(
    MSG_INIT_BASE,
    &init_base_callback,
    &init_base_node
  );
}

