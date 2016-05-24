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

#define memory_pool_t MemoryPool
#include <ch.h>
#undef memory_pool_t

#include "peripherals/leds.h"
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
#include "signal.h"
#include "system_monitor.h"
#include "main.h"
#include "sid_set.h"

/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

/** number of solution periods before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_PERIODS 2

/** number of OS ticks before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT(soln_freq_hz) MS2ST((DGNSS_TIMEOUT_PERIODS * \
  1/((float) (soln_freq_hz)) * 1000))

MemoryPool obs_buff_pool;
mailbox_t obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FIXED;

/** RTK integer ambiguity states. */
ambiguity_state_t amb_state;
/** Mutex to control access to the ambiguity states. */
MUTEX_DECL(amb_state_lock);

systime_t last_dgnss;

double soln_freq = 10.0;
u32 obs_output_divisor = 2;

double known_baseline[3] = {0, 0, 0};
u16 msg_obs_max_size = 102;

static u16 lock_counters[PLATFORM_SIGNAL_COUNT];

bool disable_raim = false;
bool send_heading = false;

void solution_send_sbp(gnss_solution *soln, dops_t *dops, bool clock_jump)
{
  if (soln) {
    /* Send GPS_TIME message first. */
    msg_gps_time_t gps_time;
    sbp_make_gps_time(&gps_time, &soln->time, 0);
    sbp_send_msg(SBP_MSG_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);
    if (chVTTimeElapsedSinceX(last_dgnss) > DGNSS_TIMEOUT(soln_freq)) {
      /* Position in LLH. */
      msg_pos_llh_t pos_llh;
      sbp_make_pos_llh(&pos_llh, soln, 0);
      sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

      /* Position in ECEF. */
      msg_pos_ecef_t pos_ecef;
      sbp_make_pos_ecef(&pos_ecef, soln, 0);
      sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);
    }
    /* Velocity in NED. */
    /* Do not send if there has been a clock jump. Velocity may be unreliable.*/
    if (!clock_jump) {
      msg_vel_ned_t vel_ned;
      sbp_make_vel_ned(&vel_ned, soln, 0);
      sbp_send_msg(SBP_MSG_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

      /* Velocity in ECEF. */
      msg_vel_ecef_t vel_ecef;
      sbp_make_vel_ecef(&vel_ecef, soln, 0);
      sbp_send_msg(SBP_MSG_VEL_ECEF, sizeof(vel_ecef), (u8 *) &vel_ecef);
    }
  }

  if (dops) {
    DO_EVERY(10,
      msg_dops_t sbp_dops;
      sbp_make_dops(&sbp_dops, dops, &(soln->time));
      sbp_send_msg(SBP_MSG_DOPS, sizeof(msg_dops_t), (u8 *) &sbp_dops);
    );
  }
}
void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm,
                        u8 fix_mode, bool clock_jump)
{
  if (chVTTimeElapsedSinceX(last_dgnss) > DGNSS_TIMEOUT(soln_freq)) {
    nmea_gpgga(soln->pos_llh, &soln->time, soln->n_used,
               fix_mode, dops->hdop, 0, 0);
  }
  nmea_send_msgs(soln, n, nm, dops, clock_jump);

}

double calc_heading(const double b_ned[3])
{
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
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
                            double ref_ecef[3], u8 flags, double hdop,
                            double corrections_age, u16 sender_id)
{
  double* base_station_pos;
  msg_baseline_ecef_t sbp_ecef;
  sbp_make_baseline_ecef(&sbp_ecef, t, n_sats, b_ecef, flags);
  sbp_send_msg(SBP_MSG_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);

  double b_ned[3];
  wgsecef2ned(b_ecef, ref_ecef, b_ned);

  msg_baseline_ned_t sbp_ned;
  sbp_make_baseline_ned(&sbp_ned, t, n_sats, b_ned, flags);
  sbp_send_msg(SBP_MSG_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);

  if (send_heading) {
    double heading = calc_heading(b_ned);
    msg_baseline_heading_t sbp_heading;
    sbp_make_heading(&sbp_heading, t, heading, n_sats, flags);
    sbp_send_msg(SBP_MSG_BASELINE_HEADING, sizeof(sbp_heading), (u8 *)&sbp_heading);
  }

  chMtxLock(&base_pos_lock);
  if (base_pos_known || (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK))) {
    last_dgnss = chVTGetSystemTime();
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];
    /* if simulation use the simulator's base station position */
    if ((simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
        simulation_enabled_for(SIMULATION_MODE_RTK))) {
      base_station_pos = simulation_ref_ecef();
    }
    else { /* else use the global variable */
      base_station_pos = base_pos_ecef;
    }

    vector_add(3, base_station_pos, b_ecef, pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);
    u8 fix_mode = (flags & 1) ? NMEA_GGA_FIX_RTK : NMEA_GGA_FIX_FLOAT;
    /* TODO: Don't fake DOP!! */
    nmea_gpgga(pseudo_absolute_llh, t, n_sats, fix_mode, hdop, corrections_age, sender_id);
    /* now send pseudo absolute sbp message */
    /* Flag in message is defined as follows :float->2, fixed->1 */
    /* We defined the flags for the SBP protocol to be spp->0, fixed->1, float->2 */
    /* TODO: Define these flags from the yaml and remove hardcoding */
    u8 sbp_flags = (flags == 1) ? 1 : 2;
    msg_pos_llh_t pos_llh;
    sbp_make_pos_llh_vect(&pos_llh, pseudo_absolute_llh, t, n_sats, sbp_flags);
    sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);
    msg_pos_ecef_t pos_ecef;
    sbp_make_pos_ecef_vect(&pos_ecef, pseudo_absolute_ecef, t, n_sats, sbp_flags);
    sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);
  }
  chMtxUnlock(&base_pos_lock);
}

static void output_baseline(u8 num_sdiffs, const sdiff_t *sdiffs,
                            const gps_time_t *t, double hdop, double diff_time, u16 base_id)
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
    chMtxUnlock(&amb_state_lock);
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
    chMtxUnlock(&amb_state_lock);
    if (ret == 1)
      log_warn("output_baseline: Float baseline RAIM repair");
    if (ret < 0) {
      log_warn("dgnss_float_baseline returned error: %d", ret);
      return;
    }
    break;
  }

  solution_send_baseline(t, num_used, b, position_solution.pos_ecef, flags, hdop, diff_time, base_id);
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
    pack_obs_header(t, total, count, (observation_header_t*) buff);
    packed_obs_content_t *obs = (packed_obs_content_t *)&buff[sizeof(observation_header_t)];

    for (u8 i = 0; i < curr_n; i++, obs_i++) {
      if (pack_obs_content(m[obs_i].raw_pseudorange,
            m[obs_i].raw_carrier_phase,
            m[obs_i].snr,
            m[obs_i].lock_counter,
            m[obs_i].sid,
            &obs[i]) < 0) {
        /* Error packing this observation, skip it. */
        i--;
        curr_n--;
      }
    }

    sbp_send_msg(SBP_MSG_OBS,
      sizeof(observation_header_t) + curr_n*sizeof(packed_obs_content_t),
      buff);

  }
}

static void solution_simulation()
{
  simulation_step();

  /* TODO: The simulator's handling of time is a bit crazy. This is a hack
   * for now but the simulator should be refactored so that it can give the
   * exact correct solution time output without this nonsense. */
  gnss_solution *soln = simulation_current_gnss_solution();
  double expected_tow = \
    round(soln->time.tow * soln_freq) / soln_freq;
  soln->time.tow = expected_tow;
  normalize_gps_time(&soln->time);

  if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
    /* Then we send fake messages. */
    solution_send_sbp(soln, simulation_current_dops_solution(), FALSE);
    solution_send_nmea(soln, simulation_current_dops_solution(),
                       simulation_current_num_sats(),
                       simulation_current_navigation_measurements(),
                       NMEA_GGA_FIX_GPS, FALSE);

  }

  if (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK)) {

    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? 1 : 0;

    solution_send_baseline(&(soln->time),
      simulation_current_num_sats(),
      simulation_current_baseline_ecef(),
      simulation_ref_ecef(), flags, 1.5, 0.25, 1023);

    double t_check = expected_tow * (soln_freq / obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      send_observations(simulation_current_num_sats(),
        &(soln->time), simulation_current_navigation_measurements());
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
    tracking_channel_evelation_degrees_set(nav_meas[i].sid, (float)el * R2D);
  }
}

/** Sleep until the next solution deadline.
 *
 * \param deadline    Pointer to the current deadline, updated by this function.
 * \param interval    Interval by which the deadline should be advanced.
 */
static void sol_thd_sleep(systime_t *deadline, systime_t interval)
{
  *deadline += interval;

  chSysLock();
  while (1) {
    /* Sleep for at least (1-SOLN_THD_CPU_MAX) * interval ticks so that
     * execution time is limited to SOLN_THD_CPU_MAX. */
    systime_t systime = chVTGetSystemTimeX();
    systime_t delta = *deadline - systime;
    systime_t sleep_min = (systime_t)ceilf((1.0f-SOLN_THD_CPU_MAX) * interval);
    if ((systime_t)(delta - sleep_min) <= ((systime_t)-1) / 2) {
      chThdSleepS(delta);
      break;
    } else {
      chSysUnlock();
      if (delta <= ((systime_t)-1) / 2) {
        /* Deadline is in the future. Skipping due to high CPU usage. */
        log_warn("Solution thread skipping deadline, "
                  "time = %lu, deadline = %lu", systime, *deadline);
      } else {
        /* Deadline is in the past. */
        log_warn("Solution thread missed deadline, "
                 "time = %lu, deadline = %lu", systime, *deadline);
      }
      *deadline += interval;
      chSysLock();
    }
  }
  chSysUnlock();
}

static WORKING_AREA_CCM(wa_solution_thread, 8000);
static void solution_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("solution");

  systime_t deadline = chVTGetSystemTimeX();

  bool clock_jump = FALSE;

  while (TRUE) {

    sol_thd_sleep(&deadline, CH_CFG_ST_FREQUENCY/soln_freq);
    watchdog_notify(WD_NOTIFY_SOLUTION);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation();
    }

    u64 rec_tc = nap_timing_count();
    gps_time_t rec_time = rx2gpstime(rec_tc);
    u8 n_ready = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    for (u8 i=0; i<nap_track_n_channels; i++) {
      tracking_channel_lock(i);
      if (use_tracking_channel(i)) {
        tracking_channel_measurement_get(i, rec_tc, &meas[n_ready]);
        n_ready++;
      }
      tracking_channel_unlock(i);
    }

    gnss_sid_set_t codes_in_track;
    sid_set_init(&codes_in_track);
    for (u8 i=0; i<n_ready; i++)
      sid_set_add(&codes_in_track, meas[i].sid);

    if (sid_set_get_sat_count(&codes_in_track) < 4) {
      /* Not enough sats, keep on looping. */
      continue;
    }

    /* Got enough sats/ephemerides, do a solution. */
    /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
     * more intelligent with the solution time.
     */
    static navigation_measurement_t nav_meas[MAX_CHANNELS];
    const channel_measurement_t *p_meas[n_ready];
    navigation_measurement_t *p_nav_meas[n_ready];
    const ephemeris_t *p_e_meas[n_ready];

    /* Create arrays of pointers for use in calc_navigation_measurement */
    for (u8 i = 0; i < n_ready; i++) {
      p_meas[i] = &meas[i];
      p_nav_meas[i] = &nav_meas[i];
      p_e_meas[i] = ephemeris_get(meas[i].sid);
    }

    /* Create navigation measurements from the channel measurements */
    ephemeris_lock();
    /* If we have timing then we can calculate the relationship between
     * receiver time and GPS time and hence provide the pseudorange
     * calculation with the local GPS time of reception. */
    /* If a FINE quality time solution is not available then don't pass in a
     * `nav_time`. This will result in valid pseudoranges but with a large
     * and arbitrary receiver clock error. We may want to discard these
     * observations after doing a PVT solution. */
    gps_time_t *p_rec_time = (time_quality == TIME_FINE) ? &rec_time : NULL;
    if (calc_navigation_measurement(n_ready, p_meas, p_nav_meas,
								                   p_rec_time, p_e_meas) != 0) {
      log_error("calc_navigation_measurement() returned an error");
      ephemeris_unlock();
      continue;
    }
    ephemeris_unlock();

    static u64 rec_tc_old = 0;
    static u8 n_ready_old = 0;
    static navigation_measurement_t nav_meas_old[MAX_CHANNELS];
    static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];
    u8 n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old,
                                   nav_meas_old, nav_meas_tdcp,
                                   (double)(rec_tc - rec_tc_old) / SAMPLE_FREQ);

    /* Store current observations for next time for
     * TDCP Doppler calculation. */
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
    n_ready_old = n_ready;
    rec_tc_old = rec_tc;

    gnss_sid_set_t codes_tdcp;
    sid_set_init(&codes_tdcp);
    for (u8 i=0; i<n_ready_tdcp; i++) {
      sid_set_add(&codes_tdcp, nav_meas_tdcp[i].sid);
    }

    if (sid_set_get_sat_count(&codes_tdcp) < 4) {
      /* Not enough sats to compute PVT */
      continue;
    }

    dops_t dops;
    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    s8 ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim,
                      &position_solution, &dops);
    if (ret < 0) {
      /* An error occurred with calc_PVT! */
      /* TODO: Make this based on time since last error instead of a simple
       * count. */
      /* pvt_err_msg defined in libswiftnav/pvt.c */
      DO_EVERY((u32)soln_freq,
        log_warn("PVT solver: %s (code %d)", pvt_err_msg[-ret-1], ret);
      );

      /* Send just the DOPs and exit the loop */
      solution_send_sbp(0, &dops, clock_jump);
      continue;
    }

    if (ret == 1)
	  log_warn("calc_PVT: RAIM repair");

    if (time_quality < TIME_FINE) {
      /* If the time quality is not FINE then our receiver clock bias isn't
       * known. We should only use this PVT solution to update our time
       * estimate and then skip all other processing.
       *
       * Note that the lack of knowledge of the receiver clock bias does NOT
       * degrade the quality of the position solution but the rapid change in
       * bias after the time estimate is first improved may cause issues for
       * e.g. carrier smoothing. Easier just to discard this first solution.
       */
      set_time_fine(rec_tc, position_solution.time);
      continue;
    }

    /* Calculate the receiver clock error and if >1ms perform a clock jump */
    double rx_err = gpsdifftime(&rec_time, &position_solution.time);
    log_debug("RX clock error = %f", rx_err);
    clock_jump = FALSE;
    if (fabs(rx_err) >= 1e-3) {
    log_info("RX clock error %f > 1ms, resetting!", rx_err);
      set_time_fine(rec_tc, position_solution.time);
      clock_jump = TRUE;
    }

    /* Update global position solution state. */
    position_updated();

    /* Save elevation angles every so often */
    DO_EVERY((u32)soln_freq,
             update_sat_elevations(nav_meas_tdcp, n_ready_tdcp,
                                   position_solution.pos_ecef));

    if (!simulation_enabled()) {
      /* Output solution. */
      solution_send_sbp(&position_solution, &dops, clock_jump);
      solution_send_nmea(&position_solution, &dops,
                         n_ready_tdcp, nav_meas_tdcp,
                         NMEA_GGA_FIX_GPS, clock_jump);
    }

    /* There are two corrections that are applied to the pseudorange,
     * the first is done to remove receiver clock error which we now know
     * (since we've performed a PVT solve).  To remove the receiver
     * clock error from the pseudorange we need to add the PVT time
     * correction term into the pseudorange.  To be more explicit, we
     * start with the pseudorange using Leick's notation:
     *
     *      P(t) = c * [t - TOT(t) + dt_k(t) - dt^p(t)]
     *
     * Where dt_k is the receiver clock error and dt^p is the satellite clock error,
     * and TOT(t) is the time of transmission for a signal that arrived at time t.
     *
     * When we first compute the raw_pseudorange (in calc_navigation_measurements)
     * dt_k(t) and dt^p(t) are unknown, so we ignore them and set:
     *
     *      P_uncorrected(tor) = P(tor) - c dt_k(tor) + c dt^p(t)
     *                         = c (tor - TOT(tor))
     *
     * Now that we know the receiver error we can get a more accurate pseudorange
     * by accounting for the receiver error (dt_k(t))
     *
     *      P_corrected(t_pvt) = P_uncorrected(tor) + c dt_k(t_pvt)
     *                         = P_uncorrected(tor) + c (t_pvt - tor)
     *                         = c (t_pvt - TOT(tor))
     *                         = c (t_pvt - TOT(t_pvt)).
     *
     *  The last bit comes from TOT(tor) == TOT(t_pvt), which simply stems from
     *  the fact that uncorrected and corrected pseudoranges correspond to the
     *  exact same observations.
     */
    for (u8 i = 0; i < n_ready_tdcp; i++) {
      nav_meas_tdcp[i].raw_pseudorange += GPS_C * gpsdifftime(&position_solution.time, &rec_time);
      nav_meas_tdcp[i].pseudorange += GPS_C * gpsdifftime(&position_solution.time, &rec_time);
    }
    /*
     * The next correction is done to create a new pseudorange that is valid for
     * a different time of arrival.  In particular we'd like to propagate all the
     * observations such that they represent what we would have observed had
     * the observations all arrived at the current epoch (t').
     */

    /* Calculate the time of the nearest solution epoch, where we expected
     * to be, and calculate how far we were away from it. */
    double expected_tow = round(position_solution.time.tow * soln_freq)
                          / soln_freq;
    double t_err = expected_tow - position_solution.time.tow;

    /* Update observation time. */
    gps_time_t new_obs_time;
    new_obs_time.wn = position_solution.time.wn;
    new_obs_time.tow = expected_tow;

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(t_err) < OBS_PROPAGATION_LIMIT) {

      /* Propagate observation to desired time. */
      /* We have to use the tdcp_doppler result to account for TCXO drift. */
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        nav_meas_tdcp[i].raw_pseudorange -= t_err;
        nav_meas_tdcp[i].raw_carrier_phase += t_err * nav_meas_tdcp[i].raw_doppler;

        nav_meas_tdcp[i].tot = new_obs_time;
        nav_meas_tdcp[i].tot.tow -= nav_meas_tdcp[i].raw_pseudorange / GPS_C;
        normalize_gps_time(&nav_meas_tdcp[i].tot);

        double clock_err;
        double clock_rate_err;
        ephemeris_lock();
        ephemeris_t *e = ephemeris_get(nav_meas_tdcp[i].sid);
        calc_sat_state(e, &nav_meas_tdcp[i].tot,
                       nav_meas_tdcp[i].sat_pos,
                       nav_meas_tdcp[i].sat_vel,
                       &clock_err, &clock_rate_err);
        ephemeris_unlock();
      }

      /* Output obervations only every obs_output_divisor times, taking
       * care to ensure that the observations are aligned. */
      /* Also only output observations once our receiver clock is
       * correctly set. */
      double t_check = expected_tow * (soln_freq / obs_output_divisor);
      if (!simulation_enabled() &&
          time_quality == TIME_FINE &&
          fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
        /* Send the observations. */
        send_observations(n_ready_tdcp, &new_obs_time, nav_meas_tdcp);
      }

      /* If we have a recent set of observations from the base station, do a
       * differential solution. */
      double pdt;
      chMtxLock(&base_obs_lock);
      if (base_obss.n > 0 && !simulation_enabled()) {
        if ((pdt = gpsdifftime(&new_obs_time, &base_obss.tor))
              < MAX_AGE_OF_DIFFERENTIAL) {

          /* Propagate base station observations to the current time and
           * process a low-latency differential solution. */

          /* Hook in low-latency filter here. */
          if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY &&
              base_obss.has_pos) {

            sdiff_t sdiffs[MAX(base_obss.n, n_ready_tdcp)];
            u8 num_sdiffs = make_propagated_sdiffs(n_ready_tdcp, nav_meas_tdcp,
                                    base_obss.n, base_obss.nm,
                                    base_obss.sat_dists, base_obss.pos_ecef,
                                    sdiffs);
            if (num_sdiffs >= 4) {
              output_baseline(num_sdiffs, sdiffs, &new_obs_time, pdt,
                              dops.hdop, base_obss.sender_id);
            }
          }
        }
      }
      chMtxUnlock(&base_obs_lock);

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
        if (ret != MSG_OK) {
          log_error("Pool full and mailbox empty!");
        }
      }
      obs->tor = new_obs_time;
      obs->n = n_ready_tdcp;
      memcpy(obs->nm, nav_meas_tdcp, obs->n * sizeof(navigation_measurement_t));
      ret = chMBPost(&obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
      if (ret != MSG_OK) {
        /* We could grab another item from the mailbox, discard it and then
         * post our obs again but if the size of the mailbox and the pool
         * are equal then we should have already handled the case where the
         * mailbox is full when we handled the case that the pool was full.
         * */
        log_error("Mailbox should have space!");
      }
    }

    /* Calculate time till the next desired solution epoch. */
    double dt = expected_tow - position_solution.time.tow;

    /* Limit dt to 1 second maximum to prevent hang if dt calculated
     * incorrectly. */
    if (fabs(dt) > 1.0) {
      dt = (dt > 0.0) ? 1.0 : -1.0;
    }

    /* Reset timer period with the count that we will estimate will being
     * us up to the next solution time. */
    deadline += dt * CH_CFG_ST_FREQUENCY;
  }
}

static bool init_done = false;
static bool init_known_base = false;
static bool reset_iar = false;

void process_matched_obs(u8 n_sds, gps_time_t *t, sdiff_t *sds, u16 base_id)
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
    chMtxUnlock(&amb_state_lock);
    /* If we are in time matched mode then calculate and output the baseline
     * for this observation. */
    if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED &&
        !simulation_enabled() && n_sds >= 4) {
      output_baseline(n_sds, sds, t, 0, 0, base_id);
    }
  }
}

static THD_WORKING_AREA(wa_time_matched_obs_thread, 20000);
static void time_matched_obs_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("time matched obs");
  while (1) {
    /* Wait for a new observation to arrive from the base station. */
    chBSemWait(&base_obs_received);

    /* Blink red LED for 20ms. */
    systime_t t_blink = chVTGetSystemTime() + MS2ST(50);
    led_on(LED_RED);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&obs_mailbox, (msg_t *)&obss, TIME_IMMEDIATE)
            == MSG_OK) {

      chMtxLock(&base_obs_lock);
      double dt = gpsdifftime(&obss->tor, &base_obss.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD) {
        /* Times match! Process obs and base_obss */
        static sdiff_t sds[MAX_CHANNELS];
        u8 n_sds = single_diff(
            obss->n, obss->nm,
            base_obss.n, base_obss.nm,
            sds
        );
        chMtxUnlock(&base_obs_lock);

        u16 *sds_lock_counters[n_sds];
        for (u32 i = 0; i < n_sds; i++) {
          sds_lock_counters[i] = &lock_counters[sid_to_global_index(sds[i].sid)];
        }

        gnss_signal_t sats_to_drop[n_sds];
        u8 num_sats_to_drop = check_lock_counters(n_sds, sds, sds_lock_counters,
                                                  sats_to_drop);
        if (num_sats_to_drop > 0) {
          /* Copies all valid sdiffs back into sds, omitting each of sats_to_drop.
           * Dropping an sdiff will cause dgnss_update to drop that sat from
           * our filters. */
          n_sds = filter_sdiffs(n_sds, sds, num_sats_to_drop, sats_to_drop);
        }
        process_matched_obs(n_sds, &obss->tor, sds, base_obss.sender_id);
        chPoolFree(&obs_buff_pool, obss);
        break;
      } else {
        chMtxUnlock(&base_obs_lock);
        if (dt > 0) {
          /* Time of base obs before time of local obs, we must not have a local
           * observation matching this base observation, break and wait for a
           * new base observation. */

          /* In practice this should basically never happen so lets make a note
           * if it does. */
          log_warn("Obs Matching: t_base < t_rover "
                   "(dt=%f obss.t={%d,%f} base_obss.t={%d,%f})", dt,
                   obss->tor.wn, obss->tor.tow,
                   base_obss.tor.wn, base_obss.tor.tow
          );
          /* Return the buffer to the mailbox so we can try it again later. */
          msg_t ret = chMBPost(&obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
          if (ret != MSG_OK) {
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
    if (t_blink > chVTGetSystemTimeX()) {
      chThdSleepS(t_blink - chVTGetSystemTimeX());
    }
    chSysUnlock();

    led_off(LED_RED);
  }
}

void reset_filters_callback(u16 sender_id, u8 len, u8 msg[], void* context)
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

void init_base_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void)context;
  init_known_base = true;
}

void solution_setup()
{
  /* Set time of last differential solution in the past. */
  last_dgnss = chVTGetSystemTime() - DGNSS_TIMEOUT(soln_freq);
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
  SETTING("solution", "send_heading", send_heading, TYPE_BOOL);

  nmea_setup();

  static msg_t obs_mailbox_buff[OBS_N_BUFF];
  chMBObjectInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);
  chPoolObjectInit(&obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[OBS_N_BUFF] _CCM;
  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread),
                    HIGHPRIO-2, solution_thread, NULL);
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
