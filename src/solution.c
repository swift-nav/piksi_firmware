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

#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/dgnss_management.h>
#include <libswiftnav/baseline.h>
#include <libswiftnav/linear_algebra.h>

#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "board/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "solution.h"
#include "manage.h"
#include "simulator.h"
#include "settings.h"
#include "timing.h"
#include "base_obs.h"
#include "ephemeris.h"
#include "./system_monitor.h"

MemoryPool obs_buff_pool;
Mailbox obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FIXED;

/** RTK integer ambiguity states. */
ambiguity_state_t amb_state;
/** Mutex to control access to the ambiguity states. */
Mutex amb_state_lock;

systime_t last_dgnss;

double soln_freq = 10.0;
u32 obs_output_divisor = 2;

double known_baseline[3] = { 0, 0, 0 };
u16 msg_obs_max_size = 104;

static u16 lock_counters[MAX_SATS];

bool disable_raim = false;

void solution_send_sbp(gnss_solution *soln, dops_t *dops)
{
  if (soln) {
    /* Send GPS_TIME message first. */
    msg_gps_time_t gps_time;
    sbp_make_gps_time(&gps_time, &soln->time, 0);
    sbp_send_msg(SBP_MSG_GPS_TIME, sizeof(gps_time), (u8 *)&gps_time);

    /* Position in LLH. */
    msg_pos_llh_t pos_llh;
    sbp_make_pos_llh(&pos_llh, soln, 0);
    sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *)&pos_llh);

    /* Position in ECEF. */
    msg_pos_ecef_t pos_ecef;
    sbp_make_pos_ecef(&pos_ecef, soln, 0);
    sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *)&pos_ecef);

    /* Velocity in NED. */
    msg_vel_ned_t vel_ned;
    sbp_make_vel_ned(&vel_ned, soln, 0);
    sbp_send_msg(SBP_MSG_VEL_NED, sizeof(vel_ned), (u8 *)&vel_ned);

    /* Velocity in ECEF. */
    msg_vel_ecef_t vel_ecef;
    sbp_make_vel_ecef(&vel_ecef, soln, 0);
    sbp_send_msg(SBP_MSG_VEL_ECEF, sizeof(vel_ecef), (u8 *)&vel_ecef);
  }

  if (dops) {
    DO_EVERY(10,
             msg_dops_t sbp_dops;
             sbp_make_dops(&sbp_dops, dops, &(soln->time));
             sbp_send_msg(SBP_MSG_DOPS, sizeof(msg_dops_t), (u8 *)&sbp_dops);
             );
  }
}
void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm,
                        u8 fix_mode)
{
  if (chTimeElapsedSince(last_dgnss) > DGNSS_TIMEOUT) {
    nmea_gpgga(soln->pos_llh, &soln->time, soln->n_used,
               fix_mode, dops->hdop);
  }
  nmea_send_msgs(soln, n, nm);

}

/** Creates and sends RTK solution.
 * If the base station position is known,
 * send the NMEA and SBP psuedo absolute msgs.
 *
 * \note this function relies upon the global base_pos_ecef and base_pos_known
 * for logic and base station position when sending psuedo absolutes.
 * If operating in simulation mode, it depends upon the simulation mode enabled
 * and the simulation base_ecef position (both available in the global struct
 * sim_settings and accessed via wrappers prototyped in simulator.h)
 *
 * \param t pointer to gps time struct representing gps time for solution
 * \param n_sats u8 representig the number of satellites
 * \param b_ecef size 3 vector of doubles representing ECEF position (meters)
 * \param ref_ecef size 3 vector of doubles representing reference position
 * for conversion from ECEF to local NED coordinates (meters)
 * \param flags u8 RTK solution flags. 1 if float, 0 if fixed
 */
void solution_send_baseline(const gps_time_t *t, u8 n_sats, double b_ecef[3],
                            double ref_ecef[3], u8 flags)
{
  double *base_station_pos;
  msg_baseline_ecef_t sbp_ecef;

  sbp_make_baseline_ecef(&sbp_ecef, t, n_sats, b_ecef, flags);
  sbp_send_msg(SBP_MSG_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);

  double b_ned[3];
  wgsecef2ned(b_ecef, ref_ecef, b_ned);

  msg_baseline_ned_t sbp_ned;
  sbp_make_baseline_ned(&sbp_ned, t, n_sats, b_ned, flags);
  sbp_send_msg(SBP_MSG_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);

  chMtxLock(&base_pos_lock);
  if (base_pos_known || (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
                         simulation_enabled_for(SIMULATION_MODE_RTK))) {
    last_dgnss = chTimeNow();
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];
    /* if simulation use the simulator's base station position */
    if ((simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
         simulation_enabled_for(SIMULATION_MODE_RTK))) {
      base_station_pos = simulation_ref_ecef();
    } else {   /* else use the global variable */
      base_station_pos = base_pos_ecef;
    }

    vector_add(3, base_station_pos, b_ecef, pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);
    u8 fix_mode = (flags & 1) ? NMEA_GGA_FIX_RTK : NMEA_GGA_FIX_FLOAT;
    /* TODO: Don't fake DOP!! */
    nmea_gpgga(pseudo_absolute_llh, t, n_sats, fix_mode, 1.5);
    /* now send pseudo absolute sbp message */
    /* Flag in message is defined as follows :float->2, fixed->1 */
    /* We defined the flags for the SBP protocol to be spp->0, fixed->1, float->2 */
    /* TODO: Define these flags from the yaml and remove hardcoding */
    u8 sbp_flags = (flags == 1) ? 1 : 2;
    msg_pos_llh_t pos_llh;
    sbp_make_pos_llh_vect(&pos_llh, pseudo_absolute_llh, t, n_sats, sbp_flags);
    sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *)&pos_llh);
    msg_pos_ecef_t pos_ecef;
    sbp_make_pos_ecef_vect(&pos_ecef, pseudo_absolute_ecef, t, n_sats,
                           sbp_flags);
    sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *)&pos_ecef);
  }
  chMtxUnlock();
}

static void output_baseline(u8 num_sdiffs, const sdiff_t *sdiffs,
                            const gps_time_t *t)
{
  double b[3];
  u8 num_used, flags;
  s8 ret;

  switch (dgnss_filter) {
  default:
  case FILTER_FIXED:
    chMtxLock(&amb_state_lock);
    ret = dgnss_baseline(num_sdiffs, sdiffs, position_solution.pos_ecef,
                         &amb_state, &num_used, b,
                         disable_raim, DEFAULT_RAIM_THRESHOLD);
    chMtxUnlock();
    if (ret > 0) {
      /* ret is <0 on error, 2 if float, 1 if fixed */
      flags = (ret == 1) ? 1 : 0;
    } else {
      log_warn("dgnss_baseline returned error: %d", ret);
      return;
    }
    break;

  case FILTER_FLOAT:
    flags = 0;
    chMtxLock(&amb_state_lock);
    ret = baseline(num_sdiffs, sdiffs, position_solution.pos_ecef,
                   &amb_state.float_ambs, &num_used, b,
                   disable_raim, DEFAULT_RAIM_THRESHOLD);
    chMtxUnlock();
    if (ret == 1) {
      log_warn("output_baseline: Float baseline RAIM repair");
    }
    if (ret < 0) {
      log_warn("dgnss_float_baseline returned error: %d", ret);
      return;
    }
    break;
  }

  solution_send_baseline(t, num_used, b, position_solution.pos_ecef, flags);
}

void send_observations(u8 n, gps_time_t *t, navigation_measurement_t *m)
{
  static u8 buff[256];

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size = MAX(
    MIN(msg_obs_max_size, SBP_FRAMING_MAX_PAYLOAD_SIZE),
    sizeof(observation_header_t)
    ) - sizeof(observation_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(packed_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(packed_obs_content_t);

  /* Round up the number of messages */
  u16 total = MIN((n + obs_in_msg - 1) / obs_in_msg, MSG_OBS_HEADER_MAX_SIZE);

  u8 obs_i = 0;
  for (u8 count = 0; count < total; count++) {

    u8 curr_n = MIN(n - obs_i, obs_in_msg);
    pack_obs_header(t, total, count, (observation_header_t *)buff);
    packed_obs_content_t *obs =
      (packed_obs_content_t *)&buff[sizeof(observation_header_t)];

    for (u8 i = 0; i < curr_n; i++, obs_i++) {
      if (pack_obs_content(m[obs_i].raw_pseudorange,
                           m[obs_i].carrier_phase,
                           m[obs_i].snr,
                           m[obs_i].lock_counter,
                           m[obs_i].prn,
                           &obs[i]) < 0) {
        /* Error packing this observation, skip it. */
        i--;
        curr_n--;
      }
    }

    sbp_send_msg(SBP_MSG_OBS,
                 sizeof(observation_header_t) + curr_n *
                 sizeof(packed_obs_content_t),
                 buff);

  }
}

static BinarySemaphore solution_wakeup_sem;
#define tim5_isr Vector108
#define NVIC_TIM5_IRQ 50
void tim5_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  /* Wake up processing thread */
  chBSemSignalI(&solution_wakeup_sem);

  timer_clear_flag(TIM5, TIM_SR_UIF);

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

static void timer_set_period_check(uint32_t timer_peripheral, uint32_t period)
{
  __asm__("CPSID i;");
  TIM_ARR(timer_peripheral) = period;
  uint32_t tmp = TIM_CNT(timer_peripheral);
  if (tmp > period) {
    TIM_CNT(timer_peripheral) = period;
    log_warn("Solution thread missed deadline, "
             "TIM counter = %lu, period = %lu", tmp, period);
  }
  __asm__("CPSIE i;");
}

static void solution_simulation(void)
{
  /* Set the timer period appropriately. */
  timer_set_period_check(TIM5, round(65472000 * (1.0 / soln_freq)));

  simulation_step();

  /* TODO: The simulator's handling of time is a bit crazy. This is a hack
   * for now but the simulator should be refactored so that it can give the
   * exact correct solution time output without this nonsense. */
  gnss_solution *soln = simulation_current_gnss_solution();
  double expected_tow = \
    round(soln->time.tow * soln_freq) / soln_freq;
  soln->time.tow = expected_tow;
  soln->time = normalize_gps_time(soln->time);

  if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
    /* Then we send fake messages. */
    solution_send_sbp(soln, simulation_current_dops_solution());
    solution_send_nmea(soln, simulation_current_dops_solution(),
                       simulation_current_num_sats(),
                       simulation_current_navigation_measurements(),
                       NMEA_GGA_FIX_GPS);

  }

  if (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK)) {

    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? 1 : 0;

    solution_send_baseline(&(soln->time),
                           simulation_current_num_sats(),
                           simulation_current_baseline_ecef(),
                           simulation_ref_ecef(), flags);

    double t_check = expected_tow * (soln_freq / obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      send_observations(
        simulation_current_num_sats(),
        &(soln->time),
        simulation_current_navigation_measurements());
    }
  }
}

/** Update the tracking channel states with satellite elevation angles
 * \param nav_meas Navigation measurements with .sat_pos populated
 * \param n_meas Number of navigation measurements
 * \param pos_ecef Receiver position
 */
static void update_sat_elevations(const navigation_measurement_t nav_meas[],
                                  u8 n_meas, const double pos_ecef[3])
{
  double _, el;

  for (int i = 0; i < n_meas; i++) {
    wgsecef2azel(nav_meas[i].sat_pos, pos_ecef, &_, &el);
    for (int j = 0; j < nap_track_n_channels; j++) {
      if (tracking_channel[j].prn == nav_meas[i].prn) {
        tracking_channel[j].elevation = (float)el * R2D;
        break;
      }
    }
  }
}

static WORKING_AREA_CCM(wa_solution_thread, 8000);
static msg_t solution_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("solution");

  static navigation_measurement_t nav_meas_old[MAX_CHANNELS];

  while (TRUE) {
    /* Waiting for the timer IRQ fire.*/
    chBSemWait(&solution_wakeup_sem);

    watchdog_notify(WD_NOTIFY_SOLUTION);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation();
    }

    u8 n_ready = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    for (u8 i = 0; i < nap_track_n_channels; i++) {
      if (use_tracking_channel(i)) {
        __asm__("CPSID i;");
        tracking_update_measurement(i, &meas[n_ready]);
        __asm__("CPSIE i;");
        n_ready++;
      }
    }

    if (n_ready < 4) {
      /* Not enough sats, keep on looping. */
      continue;
    }

    /* Got enough sats/ephemerides, do a solution. */
    /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
     * more intelligent with the solution time.
     */
    static u8 n_ready_old = 0;
    u64 nav_tc = nap_timing_count();
    static navigation_measurement_t nav_meas[MAX_CHANNELS];
    chMtxLock(&es_mutex);
    calc_navigation_measurement(n_ready, meas, nav_meas,
                                (double)((u32)nav_tc) / SAMPLE_FREQ, es);
    chMtxUnlock();

    static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];
    u8 n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old,
                                   nav_meas_old, nav_meas_tdcp);

    /* Store current observations for next time for
     * TDCP Doppler calculation. */
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
    n_ready_old = n_ready;

    if (n_ready_tdcp < 4) {
      /* Not enough sats to compute PVT */
      continue;
    }

    dops_t dops;
    s8 ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim,
                      &position_solution, &dops);
    /* disable_raim controlled by external setting. Defaults to false. */
    if (ret >= 0) {

      if (ret == 1) {
        log_warn("calc_PVT: RAIM repair");
      }

      /* Update global position solution state. */
      position_updated();
      set_time_fine(nav_tc, position_solution.time);

      /* Save elevation angles every so often */
      DO_EVERY((u32)soln_freq,
               update_sat_elevations(nav_meas_tdcp, n_ready_tdcp,
                                     position_solution.pos_ecef));

      if (!simulation_enabled()) {
        /* Output solution. */
        solution_send_sbp(&position_solution, &dops);
        solution_send_nmea(&position_solution, &dops,
                           n_ready_tdcp, nav_meas_tdcp,
                           NMEA_GGA_FIX_GPS);
      }

      /* If we have a recent set of observations from the base station, do a
       * differential solution. */
      double pdt;
      chMtxLock(&base_obs_lock);
      if (base_obss.n > 0 && !simulation_enabled()) {
        pdt = gpsdifftime(position_solution.time, base_obss.t);
        if (pdt < MAX_AGE_OF_DIFFERENTIAL) {
          /* Propagate base station observations to the current time and
           * process a low-latency differential solution. */

          /* Hook in low-latency filter here. */
          if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY &&
              base_obss.has_pos) {
            chMtxLock(&es_mutex);
            sdiff_t sdiffs[MAX(base_obss.n, n_ready_tdcp)];
            u8 num_sdiffs = make_propagated_sdiffs(n_ready_tdcp, nav_meas_tdcp,
                                                   base_obss.n, base_obss.nm,
                                                   base_obss.sat_dists,
                                                   base_obss.pos_ecef,
                                                   es, position_solution.time,
                                                   sdiffs);
            chMtxUnlock();
            if (num_sdiffs >= 4) {
              output_baseline(num_sdiffs, sdiffs, &position_solution.time);
            }
          }

        }
      }
      chMtxUnlock();

      /* Calculate the time of the nearest solution epoch, were we expected
       * to be and calculate how far we were away from it. */
      double expected_tow = round(position_solution.time.tow * soln_freq)
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
        for (u8 i = 0; i < n_ready_tdcp; i++) {
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
            log_error("Pool full and mailbox empty!");
          }
        }
        obs->t = new_obs_time;
        obs->n = n_ready_tdcp;
        memcpy(obs->nm, nav_meas_tdcp,
               obs->n * sizeof(navigation_measurement_t));
        ret = chMBPost(&obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
        if (ret != RDY_OK) {
          /* We could grab another item from the mailbox, discard it and then
           * post our obs again but if the size of the mailbox and the pool
           * are equal then we should have already handled the case where the
           * mailbox is full when we handled the case that the pool was full.
           * */
          log_error("Mailbox should have space!");
        }
      }

      /* Calculate time till the next desired solution epoch. */
      double dt = expected_tow + (1.0 / soln_freq) - position_solution.time.tow;

      /* Limit dt to 2 seconds maximum to prevent hang if dt calculated
       * incorrectly. */
      if (dt > 2) {
        dt = 2;
      }

      /* Reset timer period with the count that we will estimate will being
       * us up to the next solution time. */
      timer_set_period_check(TIM5, round(65472000 * dt));

    } else {
      /* An error occurred with calc_PVT! */
      /* TODO: Make this based on time since last error instead of a simple
       * count. */
      /* pvt_err_msg defined in libswiftnav/pvt.c */
      DO_EVERY((u32)soln_freq,
               log_warn("PVT solver: %s (code %d)", pvt_err_msg[-ret - 1], ret);
               );

      /* Send just the DOPs */
      solution_send_sbp(0, &dops);
    }
  }
  return 0;
}

static bool init_done = false;
static bool init_known_base = false;
static bool reset_iar = false;

void process_matched_obs(u8 n_sds, gps_time_t *t, sdiff_t *sds)
{
  if (init_known_base) {
    if (n_sds > 4) {
      /* Calculate ambiguities from known baseline. */
      log_info("Initializing using known baseline");
      double known_baseline_ecef[3];
      wgsned2ecef(known_baseline, position_solution.pos_ecef,
                  known_baseline_ecef);
      dgnss_init_known_baseline(n_sds, sds, position_solution.pos_ecef,
                                known_baseline_ecef);
      init_known_base = false;
    } else {
      log_warn("> 4 satellites required for known baseline init.");
    }
  }
  if (!init_done) {
    if (n_sds > 4) {
      /* Initialize filters. */
      log_info("Initializing DGNSS filters");
      dgnss_init(n_sds, sds, position_solution.pos_ecef);
      /* Initialize ambiguity states. */
      ambiguities_init(&amb_state.fixed_ambs);
      ambiguities_init(&amb_state.float_ambs);
      init_done = 1;
    }
  } else {
    if (reset_iar) {
      dgnss_reset_iar();
      reset_iar = false;
    }
    /* Update filters. */
    dgnss_update(n_sds, sds, position_solution.pos_ecef,
                 disable_raim, DEFAULT_RAIM_THRESHOLD);
    /* Update ambiguity states. */
    chMtxLock(&amb_state_lock);
    dgnss_update_ambiguity_state(&amb_state);
    chMtxUnlock();
    /* If we are in time matched mode then calculate and output the baseline
     * for this observation. */
    if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED &&
        !simulation_enabled() && n_sds >= 4) {
      output_baseline(n_sds, sds, t);
    }
  }
}

static WORKING_AREA(wa_time_matched_obs_thread, 20000);
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
        chMtxUnlock();
        u8 sats_to_drop[MAX_SATS];
        u8 num_sats_to_drop = check_lock_counters(n_sds, sds, lock_counters,
                                                  sats_to_drop);
        if (num_sats_to_drop > 0) {
          /* Copies all valid sdiffs back into sds, omitting each of sats_to_drop.
           * Dropping an sdiff will cause dgnss_update to drop that sat from
           * our filters. */
          n_sds = filter_sdiffs(n_sds, sds, num_sats_to_drop, sats_to_drop);
        }
        process_matched_obs(n_sds, &obss->t, sds);
        chPoolFree(&obs_buff_pool, obss);
        break;
      } else {
        chMtxUnlock();
        if (dt > 0) {
          /* Time of base obs before time of local obs, we must not have a local
           * observation matching this base observation, break and wait for a
           * new base observation. */

          /* In practice this should basically never happen so lets make a note
           * if it does. */
          log_warn("Obs Matching: t_base < t_rover "
                   "(dt=%f obss.t={%d,%f} base_obss.t={%d,%f})", dt,
                   obss->t.wn, obss->t.tow,
                   base_obss.t.wn, base_obss.t.tow
                   );
          /* Return the buffer to the mailbox so we can try it again later. */
          msg_t ret = chMBPost(&obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
          if (ret != RDY_OK) {
            /* Something went wrong with returning it to the buffer, better just
             * free it and carry on. */
            log_warn("Obs Matching: mailbox full, discarding observation!");
            chPoolFree(&obs_buff_pool, obss);
          }
          break;
        } else {
          /* Time of base obs later than time of local obs,
           * keep moving through the mailbox. */
          chPoolFree(&obs_buff_pool, obss);
        }
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

void reset_filters_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id; (void)len; (void)context;
  switch (msg[0]) {
  case 0:
    log_info("Filter reset requested");
    init_done = false;
    break;
  case 1:
    log_info("IAR reset requested");
    reset_iar = true;
    break;
  default:
    break;
  }
}

void init_base_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id; (void)len; (void)msg; (void)context;
  init_known_base = true;
}

void solution_setup(void)
{
  /* Set time of last differential solution in the past. */
  last_dgnss = chTimeNow() - DGNSS_TIMEOUT;

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

  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);

  nmea_setup();

  static msg_t obs_mailbox_buff[OBS_N_BUFF];
  chMBInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);
  chPoolInit(&obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[OBS_N_BUFF] _CCM;
  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  chMtxInit(&amb_state_lock);

  /* Initialise solution thread wakeup semaphore */
  chBSemInit(&solution_wakeup_sem, TRUE);
  /* Start solution thread */
  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread),
                    HIGHPRIO - 2, solution_thread, NULL);
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvicEnableVector(NVIC_TIM5_IRQ,
                   CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY + 1));
  timer_reset(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, 0);
  timer_disable_preload(TIM5);
  timer_set_period(TIM5, 65472000); /* 1 second. */
  timer_enable_counter(TIM5);
  timer_enable_irq(TIM5, TIM_DIER_UIE);

  chThdCreateStatic(wa_time_matched_obs_thread,
                    sizeof(wa_time_matched_obs_thread), LOWPRIO,
                    time_matched_obs_thread, NULL);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
    SBP_MSG_RESET_FILTERS,
    &reset_filters_callback,
    &reset_filters_node
    );

  static sbp_msg_callbacks_node_t init_base_node;
  sbp_register_cbk(
    SBP_MSG_INIT_BASE,
    &init_base_callback,
    &init_base_node
    );
}
