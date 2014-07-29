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
#include "timing.h"

Mutex base_obs_lock;
BinarySemaphore base_obs_received;
MemoryPool obs_buff_pool;
Mailbox obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_TIME_MATCHED;
dgnss_filter_t dgnss_filter = FILTER_FIXED;

double soln_freq = 10.0;
u32 obs_output_divisor = 2;

double known_baseline[3] = {0, 0, 0};
u16 msg_obs_max_size = 104;

void solution_send_sbp(gnss_solution *soln, dops_t *dops)
{
  if (soln) {
    /* Send GPS_TIME message first. */
    sbp_gps_time_t gps_time;
    sbp_make_gps_time(&gps_time, &soln->time, 0);
    sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

    /* Position in LLH. */
    sbp_pos_llh_t pos_llh;
    sbp_make_pos_llh(&pos_llh, soln, 0);
    sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

    /* Position in ECEF. */
    sbp_pos_ecef_t pos_ecef;
    sbp_make_pos_ecef(&pos_ecef, soln, 0);
    sbp_send_msg(SBP_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);

    /* Velocity in NED. */
    sbp_vel_ned_t vel_ned;
    sbp_make_vel_ned(&vel_ned, soln, 0);
    sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

    /* Velocity in ECEF. */
    sbp_vel_ecef_t vel_ecef;
    sbp_make_vel_ecef(&vel_ecef, soln, 0);
    sbp_send_msg(SBP_VEL_ECEF, sizeof(vel_ecef), (u8 *) &vel_ecef);
  }

  if (dops) {
    DO_EVERY(10,
      sbp_dops_t sbp_dops;
      sbp_make_dops(&sbp_dops, dops);
      sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
    );
  }
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
  sbp_baseline_ecef_t sbp_ecef;
  sbp_make_baseline_ecef(&sbp_ecef, t, n_sats, b_ecef, flags);
  sbp_send_msg(SBP_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);

  double b_ned[3];
  wgsecef2ned(b_ecef, ref_ecef, b_ned);

  sbp_baseline_ned_t sbp_ned;
  sbp_make_baseline_ned(&sbp_ned, t, n_sats, b_ned, flags);
  sbp_send_msg(SBP_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);
}

extern ephemeris_t es[MAX_SATS];

obss_t base_obss;

void obs_old_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;
  (void) len;
  (void) msg;
  (void) sender_id;

  printf("Receiving an old deprecated observation message.\n");
}
void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  static s16 prev_count = 0;
  static gps_time_t prev_t = {.tow = 0.0, .wn = 0};

  /* Sender ID of zero means that the messages are relayed observations,
   * ignore them. */
  if (sender_id == 0)
    return;

  /* Relay observations using sender_if = 0. */
  sbp_send_msg_(MSG_PACKED_OBS, len, msg, 0);

  gps_time_t t;
  u8 total;
  u8 count;
  unpack_obs_header((msg_obs_header_t*)msg, &t, &total, &count);
  double epoch_count = t.tow * (soln_freq / obs_output_divisor);

  if (fabs(epoch_count - round(epoch_count)) > TIME_MATCH_THRESHOLD) {
    printf("Unaligned observation from base station ignored.\n");
    return;
  }

  /* Calculate packet latency */
  if (time_quality >= TIME_COARSE) {
    gps_time_t now = get_current_time();
    float latency_ms = (float) ((now.tow - t.tow) * 1000.0);

    log_obs_latency(latency_ms);
  }

  static u8 obss_i = 0;

  /* Verify sequence integrity */
  if (count == 0) {
    prev_t = t;
    prev_count = 0;
  } else if (prev_t.tow != t.tow ||
        prev_t.wn != t.wn ||
        prev_count + 1 != count) {
      printf("Dropped one of the observation packets! Skipping this sequence.\n");
      prev_count = -1;
      return;
  } else {
      prev_count = count;
  }

  u8 obs_in_msg = (len - sizeof(msg_obs_header_t)) / sizeof(msg_obs_content_t);

  /* Lock mutex before modifying base_obss. */
  chMtxLock(&base_obs_lock);

  if (count == 0) {
    obss_i = 0;
    base_obss.n = obs_in_msg;
    base_obss.t = t;
  } else {
    base_obss.n += obs_in_msg;
  }
  msg_obs_content_t *obs = (msg_obs_content_t *)(msg + sizeof(msg_obs_header_t));
  for (u8 i=0; i<obs_in_msg; i++, obss_i++) {
    unpack_obs_content(&obs[i],
      &base_obss.nm[obss_i].raw_pseudorange,
      &base_obss.nm[obss_i].carrier_phase,
      &base_obss.nm[obss_i].snr,
      &base_obss.nm[obss_i].prn);
  }

  /* Ensure observations sorted by PRN. */
  qsort(base_obss.nm, base_obss.n,
        sizeof(navigation_measurement_t), nav_meas_cmp);

  /* Unlock mutex. */
  chMtxUnlock();

  if (count == total - 1) {
    /* Signal that a complete base observation has been received. */
    chBSemSignal(&base_obs_received);
  }
}

void send_observations(u8 n, gps_time_t *t, navigation_measurement_t *m)
{
  static u8 buff[256];

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size = MAX(
      MIN(msg_obs_max_size, SBP_FRAMING_MAX_PAYLOAD_SIZE),
      sizeof(msg_obs_header_t)
    ) - sizeof(msg_obs_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(msg_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(msg_obs_content_t);

  /* Round up the number of messages */
  u16 total = (n + obs_in_msg - 1) / obs_in_msg;

  if (total > MSG_OBS_HEADER_MAX_SIZE) {
    printf("Capping number of observations sent\n");
    total = MSG_OBS_HEADER_MAX_SIZE;
  }

  u8 obs_i = 0;
  for (u8 count = 0; count < total; count++) {

    u8 curr_n = MIN(n - obs_i, obs_in_msg);
    pack_obs_header(t, total, count, (msg_obs_header_t*) buff);
    msg_obs_content_t *obs = (msg_obs_content_t *)&buff[sizeof(msg_obs_header_t)];

    for (u8 i = 0; i < curr_n; i++, obs_i++) {
      pack_obs_content(m[obs_i].raw_pseudorange,
        m[obs_i].carrier_phase,
        m[obs_i].snr,
        m[obs_i].prn,
        &obs[i]);
    }

    sbp_send_msg(MSG_PACKED_OBS,
      sizeof(msg_obs_header_t) + curr_n*sizeof(msg_obs_content_t),
      buff);

  }

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
      s8 ret;
      if ((ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp,
                          &position_solution, &dops)) == 0) {

        /* Update global position solution state. */
        position_updated();
        set_time_fine(nav_tc, position_solution.time);

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
        double expected_tow = round(position_solution.time.tow*soln_freq)
                                / soln_freq;
        double t_err = expected_tow - position_solution.time.tow;

        /* Only send observations that are closely aligned with the desired
         * solution epochs to ensure they haven't been propagated too far. */
        /* Output obervations only every obs_output_divisor times, taking
         * care to ensure that the observations are aligned. */
        double t_check = expected_tow * (soln_freq / obs_output_divisor);
        if (fabs(t_err) < OBS_PROPAGATION_LIMIT &&
            fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
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
        double dt = expected_tow + (1.0/soln_freq) - position_solution.time.tow;

        /* Limit dt to 2 seconds maximum to prevent hang if dt calculated
         * incorrectly. */
        if (dt > 2)
          dt = 2;

        /* Reset timer period with the count that we will estimate will being
         * us up to the next solution time. */
        timer_set_period(TIM5, round(65472000 * dt));

      } else {
        /* An error occurred with calc_PVT! */
        /* TODO: Move these error messages into libswiftnav. */
        static const char *err_msg[] = {
          "PDOP too high",
          "Altitude unreasonable",
          "ITAR lockout",
          "Took too long to converge",
        };
        /* TODO: Make this based on time since last error instead of a simple
         * count. */
        DO_EVERY((u32)soln_freq,
          printf("PVT solver: %s (%d)\n", err_msg[-ret-1], ret);
        );

        /* Send just the DOPs */
        solution_send_sbp(0, &dops);
      }

    }

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {

      /* Set the timer period appropriately. */
      timer_set_period(TIM5, round(65472000 * (1.0/soln_freq)));

      simulation_step();

      if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
        /* Then we send fake messages. */
        solution_send_sbp(simulation_current_gnss_solution(),
                          simulation_current_dops_solution());
      }

      double expected_tow = \
        round(simulation_current_gnss_solution()->time.tow * soln_freq) / soln_freq;
      double t_check = expected_tow * (soln_freq / obs_output_divisor);

      if ((simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
           simulation_enabled_for(SIMULATION_MODE_RTK)) &&
          fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {

        u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? 1 : 0;

        solution_send_baseline(&simulation_current_gnss_solution()->time,
          simulation_current_num_sats(),
          simulation_current_baseline_ecef(),
          simulation_ref_ecef(), flags);

        send_observations(simulation_current_num_sats(),
          &simulation_current_gnss_solution()->time,
          simulation_current_navigation_measurements());

        if (simulation_enabled_for(SIMULATION_MODE_RTK)) {
          msg_iar_state_t iar_state = { .num_hyps = 1 };
          sbp_send_msg(MSG_IAR_STATE, sizeof(msg_iar_state_t), (u8 *)&iar_state);
        }
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

  if (init_known_base) {
    if (n_sds > 4) {
      /* Calculate ambiguities from known baseline. */
      printf("Initializing using known baseline\n");
      double known_baseline_ecef[3];
      wgsned2ecef(known_baseline, position_solution.pos_ecef,
                  known_baseline_ecef);
      dgnss_init_known_baseline(n_sds, sds, position_solution.pos_ecef,
                                known_baseline_ecef);
      init_known_base = false;
    } else {
      printf("> 4 satellites required for known baseline init.\n");
    }
  }
  if (!init_done) {
    if (n_sds > 4) {
      /* Initialize filters. */
      printf("Initializing DGNSS filters\n");
      dgnss_init(n_sds, sds, position_solution.pos_ecef, dt);
      init_done = 1;
    }
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
        dgnss_fixed_baseline2(n_sds, sds, position_solution.pos_ecef,
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
        process_matched_obs(n_sds, &obss->t, sds, 1.0 / soln_freq);
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

  SETTING("solution", "soln_freq", soln_freq, TYPE_FLOAT);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);

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

  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

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

  static sbp_msg_callbacks_node_t obs_old_node;
  sbp_register_cbk(
    MSG_OLD_OBS,
    &obs_old_callback,
    &obs_old_node
  );

  static sbp_msg_callbacks_node_t obs_packed_node;
  sbp_register_cbk(
    MSG_PACKED_OBS,
    &obs_callback,
    &obs_packed_node
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

