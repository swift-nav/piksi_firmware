/*
 * Copyright (C) 2012-2014, 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <float.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/track.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/time.h>
#include <ch.h>
#include <track.h>
#include <assert.h>

#include "settings.h"

#include "simulator.h"
#include "solution.h"
#include "peripherals/leds.h"
#include "sbp.h"
#include "sbp_utils.h"

#include "simulator_data.h"

/** \simulator
 * \{ */

u8 sim_enabled;

simulation_settings_t sim_settings = {
  .base_ecef = {
    -2700303.10144031,
    -4292474.39651309,
    3855434.34087421
  },
  .speed = 4.0,
  .radius = 100.0,
  .pos_sigma = 1.5,
  .speed_sigma = .15,
  .cn0_sigma = 0.3,
  .pseudorange_sigma = 4,
  .phase_sigma = 3e-2,
  .num_sats = 9,
  .mode_mask =
    SIMULATION_MODE_PVT |
    SIMULATION_MODE_TRACKING |
    SIMULATION_MODE_FLOAT |
    SIMULATION_MODE_RTK
};

/* Internal Simulation State Definition */
struct {

  u32            last_update_ticks;       /**< The last simulation update happened at this CPU tick count. */
  float          angle;                   /**< Current simulation angle in radians */
  double         pos[3];                  /**< Current simulated position with no noise, in ECEF coordinates. */
  double         baseline[3];             /**< Current simulated baseline with no noise, in ECEF coordinates.*/

  u8             num_sats_selected;

  tracking_channel_state_t  tracking_channel[MAX_CHANNELS];
  navigation_measurement_t  nav_meas[MAX_CHANNELS];
  navigation_measurement_t  base_nav_meas[MAX_CHANNELS];
  dops_t                    dops;
  gnss_solution             noisy_solution;

} sim_state = {

  .last_update_ticks = 0,
  .angle = 0.0,
  .pos = {
    0.0,
    0.0,
    0.0
  },
  .baseline = {
    0.0,
    0.0,
    0.0
  },

  .num_sats_selected = 0,

  /* .tracking_channel left uninitialized */
  /* .nav_meas left uninitialized */
  /* .base_nav_meas left uninitialized */
  .dops = {
    .pdop = 1.9,
    .gdop = 1.8,
    .tdop = 1.7,
    .hdop = 1.6,
    .vdop = 1.5,
  },
  /* .noisy_solution left uninitialized */

};


/** Generates a sample from the normal distribution
* with given variance.
*
* Uses the Box-Muller transform which is insensitive
* to the long tail of gaussians.
*
* Performs a square-root, a sin, a log, and a rand call.
*
* \param variance The variance of a zero-mean gaussian to draw a sample from.
*/
double rand_gaussian(const double variance)
{
  static bool hasSpare = false;
  static double rand1, rand2;

  if(hasSpare)
  {
    hasSpare = false;
    return sqrt(variance * rand1) * sin(rand2);
  }

  hasSpare = true;

  rand1 = rand() / ((double) RAND_MAX);
  if(rand1 < FLT_MIN)
    rand1 = FLT_MIN;
  rand1 = -2 * log(rand1);
  rand2 = (rand() / ((double) RAND_MAX)) * (M_PI*2.0);

  return sqrt(variance * rand1) * cos(rand2);
}

/** Performs a 1D linear interpolation from a point on the line segment
* defined between points U and V, into a point on the line segment defined by
* points X and Y.
*
* Assumes V > U, and Y > X
*
*
* \param t The parametric variable ranging from [0,1] along [u,v]
* \param u The start of the input range.
* \param v The end of the input range.
* \param x The start of the output range.
* \param y The end of the output range.
*/
double lerp(double t, double u, double v, double x, double y) {
  return (t - u) / (v - u) * (y - x) + x;
}


/** Performs a timestep of the simulation that flies in a circle around a point.
* Updates the sim_state and sim_state.noisy_solution structs.
*
* This simulator models a system moving in a perfect circle. We use this fact to
* write a simple but smart numerically stable simulator.
*
* At every step, this simulator runs a simple forward euler integrator
* on the position of the simulated point. This new position will not be
* on the circular path we want to follow (an example numerical instability).
* To avoid numerical instability, this simulator makes a small angle
* approximation using this new position and the circle's desired
* radius to calculate the new angle around the circle the point actually is.
* This is stored in a single system variable "angle".
* "angle" wraps around 2*PI, and is used to calculate the new position.
*
* We use the angle variable to calculate a new position
*
* Adds IID gaussian noise to the true position calculated at every timestep.
*
* This function makes a small angle approximation, so the
* elapsed time (dt) between calls must be such that the (speed * dt) is much less than the radius.
*
*/
void simulation_step(void)
{

  //First we propagate the current fake PVT solution
  systime_t now_ticks = chVTGetSystemTime();

  double elapsed = (now_ticks - sim_state.last_update_ticks)/(double)CH_CFG_ST_FREQUENCY;
  sim_state.last_update_ticks = now_ticks;

  /* Update the time */
  sim_state.noisy_solution.time.tow += elapsed;

  simulation_step_position_in_circle(elapsed);
  simulation_step_tracking_and_observations(elapsed);

}

/**
* Performs a simulation step for the given duration, by moving
* our simulated position in a circle at a given radius and speed
* around the simulation's center point.
*/
void simulation_step_position_in_circle(double elapsed)
{

  /* Update the angle, making a small angle approximation. */
  sim_state.angle += (sim_settings.speed * elapsed) / sim_settings.radius;
  if (sim_state.angle > 2*M_PI) {
    sim_state.angle = 0;
  }

  double pos_ned[3] = {
    sim_settings.radius * sin(sim_state.angle),
    sim_settings.radius * cos(sim_state.angle),
    0
  };

  /* Fill out position simulation's gnss_solution pos_ECEF, pos_LLH structures */
  wgsned2ecef_d(pos_ned,
    sim_settings.base_ecef,
    sim_state.pos);

  /* Calculate an accurate baseline for simulating RTK */
  vector_subtract(3,
    sim_state.pos,
    sim_settings.base_ecef,
    sim_state.baseline);

  /* Add gaussian noise to PVT position */
  double* pos_ecef = sim_state.noisy_solution.pos_ecef;
  double pos_variance = sim_settings.pos_sigma * sim_settings.pos_sigma;
  pos_ecef[0] = sim_state.pos[0] + rand_gaussian(pos_variance);
  pos_ecef[1] = sim_state.pos[1] + rand_gaussian(pos_variance);
  pos_ecef[2] = sim_state.pos[2] + rand_gaussian(pos_variance);

  wgsecef2llh(sim_state.noisy_solution.pos_ecef, sim_state.noisy_solution.pos_llh);

  /* Calculate Velocity vector tangent to the sphere */
  double noisy_speed = sim_settings.speed +
                       rand_gaussian(sim_settings.speed_sigma *
                                     sim_settings.speed_sigma);

  sim_state.noisy_solution.vel_ned[0] = noisy_speed * cos(sim_state.angle);
  sim_state.noisy_solution.vel_ned[1] = noisy_speed * -1.0 * sin(sim_state.angle);
  sim_state.noisy_solution.vel_ned[2] = 0.0;

  wgsned2ecef(sim_state.noisy_solution.vel_ned,
    sim_state.noisy_solution.pos_ecef,
    sim_state.noisy_solution.vel_ecef);
}

/** Simulates real observations for the current position and the satellite almanac and week
* given in simulator_data.
*
* NOTES:
*
* - This simulates the pseudorange as the true distance to the satellite + noise.
* - This simulates the carrier phase as the true distance in wavelengths + bais + noise.
* - The bias is an integer multiple of 10 for easy debugging.
* - The satellite SNR/CN0 is proportional to the elevation of the satellite.
*
* USES:
* - Pipe observations into internals for testing
* - For integration testing with other devices that has to carry the radio signal.
*
* \param elapsed Number of seconds elapsed since last simulation step.
*/
void simulation_step_tracking_and_observations(double elapsed)
{
  (void)elapsed;

  gps_time_t t = sim_state.noisy_solution.time;

  /* First we calculate all the current sat positions, velocities */
  for (u8 i=0; i<simulation_num_almanacs; i++) {
    double clock_err, clock_rate_err;
    s8 r = calc_sat_state_almanac(&simulation_almanacs[i], &t,
                                  simulation_sats_pos[i],
                                  simulation_sats_vel[i],
                                  &clock_err, &clock_rate_err);
    assert(r == 0);
  }


  /* Calculate the first sim_settings.num_sats amount of visible sats */
  u8 num_sats_selected = 0;
  double az, el;
  for (u8 i=0; i<simulation_num_almanacs; i++) {
    s8 r = calc_sat_az_el_almanac(&simulation_almanacs[i], &t,
                                  sim_state.pos, &az, &el);

    assert(r == 0);
    if (el > 0 &&
        num_sats_selected < sim_settings.num_sats &&
        num_sats_selected < MAX_CHANNELS) {

      /* Generate a code measurement which is just the pseudorange: */
      double points_to_sat[3];
      double base_points_to_sat[3];

      vector_subtract(3, simulation_sats_pos[i], sim_state.pos, points_to_sat);
      vector_subtract(3, simulation_sats_pos[i], sim_settings.base_ecef, base_points_to_sat);

      double distance_to_sat = vector_norm(3, points_to_sat);
      double base_distance_to_sat = vector_norm(3, base_points_to_sat);

      /* Fill out the observation details into the NAV_MEAS structure for this satellite, */
      /* We simulate the pseudorange as a noisy range measurement, and */
      /* the carrier phase as a noisy range in wavelengths + an integer offset. */

      populate_nav_meas(&sim_state.nav_meas[num_sats_selected],
        distance_to_sat, el, i);

      populate_nav_meas(&sim_state.base_nav_meas[num_sats_selected],
        base_distance_to_sat, el, i);

      /* As for tracking, we just set each sat consecutively in each channel. */
      /* This will cause weird jumps when a satellite rises or sets. */
      gnss_signal_t sid = {
        .code = simulation_almanacs[i].sid.code,
        .sat = simulation_almanacs[i].sid.sat + SIM_PRN_OFFSET
      };
      sim_state.tracking_channel[num_sats_selected].state = 1;
      sim_state.tracking_channel[num_sats_selected].sid = sid_to_sbp(sid);
      sim_state.tracking_channel[num_sats_selected].cn0 = sim_state.nav_meas[num_sats_selected].snr;

      num_sats_selected++;
    }
  }

  sim_state.noisy_solution.n_used = num_sats_selected;

}

/** Populate a navigation_measurement_t structure with simulated data for
* the almanac_i satellite, currently dist away from simulated point at given elevation.
*
*/
void populate_nav_meas(navigation_measurement_t *nav_meas, double dist, double elevation, int almanac_i)
{
  nav_meas->sid = (gnss_signal_t) {
    .code = simulation_almanacs[almanac_i].sid.code,
    .sat = simulation_almanacs[almanac_i].sid.sat + SIM_PRN_OFFSET
  };

  nav_meas->raw_pseudorange =  dist;
  nav_meas->raw_pseudorange += rand_gaussian(sim_settings.pseudorange_sigma *
                                             sim_settings.pseudorange_sigma);

  nav_meas->raw_carrier_phase =     dist / (GPS_C /
            code_to_carr_freq(simulation_almanacs[almanac_i].sid.code));
  nav_meas->raw_carrier_phase +=   simulation_fake_carrier_bias[almanac_i];
  nav_meas->raw_carrier_phase +=   rand_gaussian(sim_settings.phase_sigma *
                                             sim_settings.phase_sigma);

  nav_meas->snr             =  lerp(elevation, 0, M_PI/2, 35, 45) +
                               rand_gaussian(sim_settings.cn0_sigma *
                                             sim_settings.cn0_sigma);
}

/** Returns true if the simulation is at all enabled
*/
inline bool simulation_enabled(void)
{
  return (sim_enabled > 0);
}

/** Returns true fi the simulation is enabled for the given mode_mask
*
* \param mode_mask The mode for which the simulation might be enabled.
*/
inline bool simulation_enabled_for(simulation_modes_t mode_mask) {
  return (sim_enabled > 0) &&
    ((sim_settings.mode_mask & mode_mask) > 0);
}

/** Get current simulated PVT solution
* The structure returned by this changes every time simulation_step is called.
*/
inline gnss_solution* simulation_current_gnss_solution(void)
{
  return &sim_state.noisy_solution;
}

/** Get current simulated DOPS.
* The structure returned by this changes when settings are updated.
*/
inline dops_t* simulation_current_dops_solution(void)
{
  return &sim_state.dops;
}

/** Get current simulated baseline reference point in ECEF coordinates.
* The structure returned by this changes when settings are updated.
*/
inline double* simulation_ref_ecef(void)
{
  return sim_settings.base_ecef;
}

/** Get current simulated baseline vector in ECEF coordinates.
* The structure returned by this changes every time simulation_step is called.
*/
inline double* simulation_current_baseline_ecef(void)
{
  return sim_state.baseline;
}

/** Returns the number of satellites being simulated.
*/
u8 simulation_current_num_sats(void)
{
  return sim_state.noisy_solution.n_used;
}

/** Returns the current simulated tracking loops state simulated.
* This contains only noise, no interesting simulation information.
*
* \param channel The simulated tracking channel.
*/
tracking_channel_state_t simulation_current_tracking_state(u8 channel)
{
  if (channel >= simulation_current_num_sats()) {
    channel = simulation_current_num_sats() - 1;
  }
  return sim_state.tracking_channel[channel];
}

/** Returns the simulated navigation measurement of our moving position.
*/
navigation_measurement_t* simulation_current_navigation_measurements(void)
{
  return sim_state.nav_meas;
}

/** Returns the simulated navigation measurement at the base position
* for the simulation (aka the non-moving point around which the simulation moves).
* This is useful for testing RTK algorithms in hardware.
*/
navigation_measurement_t* simulation_current_base_navigation_measurements(void)
{
  return sim_state.base_nav_meas;
}

/**
* Do any setup we need for the satellite almanacs.
*/
void simulator_setup_almanacs(void)
{
  for (u8 i = 0; i < simulation_num_almanacs; i++) {
    simulation_fake_carrier_bias[i] = (rand() % 1000) * 10;
  }
}

/** Must be called from main() or equivalent function before simulator runs
*/
void simulator_setup(void)
{
  sim_state.noisy_solution.time.wn = simulation_week_number;
  sim_state.noisy_solution.time.tow = 0;

  simulator_setup_almanacs();

  SETTING("simulator", "enabled",           sim_enabled,                    TYPE_BOOL);
  SETTING("simulator", "base_ecef_x",       sim_settings.base_ecef[0],      TYPE_FLOAT);
  SETTING("simulator", "base_ecef_y",       sim_settings.base_ecef[1],      TYPE_FLOAT);
  SETTING("simulator", "base_ecef_z",       sim_settings.base_ecef[2],      TYPE_FLOAT);
  SETTING("simulator", "speed",             sim_settings.speed,             TYPE_FLOAT);
  SETTING("simulator", "radius",            sim_settings.radius,            TYPE_FLOAT);
  SETTING("simulator", "pos_sigma",         sim_settings.pos_sigma,         TYPE_FLOAT);
  SETTING("simulator", "speed_sigma",       sim_settings.speed_sigma,       TYPE_FLOAT);
  SETTING("simulator", "cn0_sigma",         sim_settings.cn0_sigma,         TYPE_FLOAT);
  SETTING("simulator", "pseudorange_sigma", sim_settings.pseudorange_sigma, TYPE_FLOAT);
  SETTING("simulator", "phase_sigma",       sim_settings.phase_sigma,       TYPE_FLOAT);
  SETTING("simulator", "num_sats",          sim_settings.num_sats,          TYPE_INT);
  SETTING("simulator", "mode_mask",         sim_settings.mode_mask,         TYPE_INT);
}

/** \} */
