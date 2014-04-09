/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
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

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/track.h>
#include <libswiftnav/almanac.h>
#include <ch.h>
#include <track.h>

#include "simulator.h"
#include "solution.h"
#include "sbp_piksi.h"
#include "board/leds.h"
#include "sbp.h"

#include "simulator_data.h"

/** \simulator 
 * \{ */

simulation_settings_t simulation_settings = {
  .center_ecef = {-2700303.10144031,-4292474.39651309,3855434.34087421},
  .speed = 4.0,
  .radius = 100.0,
  .pos_variance = 2.0,
  .speed_variance = 0.02,
  .tracking_cn0_variance = 0.1,
  .pseudorange_variance = 16,
  .carrier_phase_variance = 9e-4,
  .num_sats = 9,
  .enabled = 0,
};

simulation_state_t simulation_state = {
    .last_update_ticks = 0,
    .current_angle_rad = 0.0,
    .true_pos_ecef = {0.0, 0.0, 0.0},
    .true_baseline_ecef = {0.0,0.0,0.0},

    .num_sats_selected = 0,

    .dops = {
      .pdop = 1.9,
      .gdop = 1.8,
      .tdop = 1.7,
      .hdop = 1.6,
      .vdop = 1.5,
    },

};

#define DEBUGGING 1
#if DEBUGGING
  #define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
  #define Debug(fmt, args ...)
#endif

#define Notify(fmt, args ...)  do {printf("Piksi: " fmt "\n", ## args); } while(0)

/** Generates a sample from the normal distribution 
* with given variance.
*
* Uses the Box-Muller transform which is insensitive
* to the long tail of gaussians.
*
* Performs a square-root, a sin, a log, and a rand call.
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
  if(rand1 < 1e-100) rand1 = 1e-100;
  rand1 = -2 * log(rand1);
  rand2 = (rand() / ((double) RAND_MAX)) * (M_PI*2.0);
 
  return sqrt(variance * rand1) * cos(rand2);
}

/** Performs a 1D linear interpolation from a point on the line segment
* defined between points U and V, into a point on the line segment defined by
* points X and Y.
*
* Assumes V > U, and Y > X
*/
double lerp(double t, double u, double v, double x, double y) {
  return (t - u) / (v - u) * (y - x) + x;
}


/** Performs a timestep of the simulation that flies in a circle around a point.
* Updates the simulation_state and simulation_state.noisy_solution structs.
*
* This simulator models a system moving in a perfect circle. We use this fact to
* write a simple but smart numerically stable simulator.
*
* At every step, this simulator runs a simple forward euler integrator on the position of 
* the simulated point. This new position will not be on the circular path we want to follow
* (an example numerical instability). To avoid numerical instability, 
* this simulator makes a small angle approximation using this new position and the circle's desired
* radius to calculate the new angle around the circle the point actually is.
* This is stored in a single system variable "current_angle_rad".
* "current_angle_rad" wraps around 2*PI, and is used to calculate the new position.
* 
* We use the current_angle_rad variable to calculate a new position 
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
  u32 now_ticks = chTimeNow();
  
  double elapsed_seconds = (now_ticks - simulation_state.last_update_ticks)/(double)CH_FREQUENCY;
  simulation_state.last_update_ticks = now_ticks;

  //Update the time
  simulation_state.noisy_solution.time.tow += elapsed_seconds;

  simulation_step_position_in_circle(elapsed_seconds);
  simulation_step_tracking_and_observations(elapsed_seconds);

}

void simulation_step_position_in_circle(double elapsed_seconds) 
{
  //Update the angle, making a small angle approximation.
  simulation_state.current_angle_rad += (simulation_settings.speed * elapsed_seconds) / simulation_settings.radius;
  if (simulation_state.current_angle_rad > 2*M_PI) {
    simulation_state.current_angle_rad = 0;
  }

  double pos_ned[3] = { 
    simulation_settings.radius * sin(simulation_state.current_angle_rad),
    simulation_settings.radius * cos(simulation_state.current_angle_rad),
    0
  };

  //Fill out position simulation's gnss_solution pos_ECEF, pos_LLH structures
  wgsned2ecef_d(pos_ned, simulation_settings.center_ecef, simulation_state.true_pos_ecef);

  //Calculate an accurate baseline for simulating RTK
  vector_subtract(3, simulation_state.true_pos_ecef, simulation_settings.center_ecef, simulation_state.true_baseline_ecef);

  // //Add gaussian noise to PVT position
  simulation_state.noisy_solution.pos_ecef[0] = simulation_state.true_pos_ecef[0] + rand_gaussian(simulation_settings.pos_variance);
  simulation_state.noisy_solution.pos_ecef[1] = simulation_state.true_pos_ecef[1] + rand_gaussian(simulation_settings.pos_variance);
  simulation_state.noisy_solution.pos_ecef[2] = simulation_state.true_pos_ecef[2] + rand_gaussian(simulation_settings.pos_variance);
  
  wgsecef2llh(simulation_state.noisy_solution.pos_ecef, simulation_state.noisy_solution.pos_llh);

  //Calculate Velocity vector tangent to the sphere
  double noisy_speed = simulation_settings.speed + rand_gaussian(simulation_settings.speed_variance);

  simulation_state.noisy_solution.vel_ned[0] = noisy_speed * cos(simulation_state.current_angle_rad);
  simulation_state.noisy_solution.vel_ned[1] = noisy_speed * -1.0 * sin(simulation_state.current_angle_rad);
  simulation_state.noisy_solution.vel_ned[2] = 0.0;

  wgsned2ecef(simulation_state.noisy_solution.vel_ned, simulation_state.noisy_solution.pos_ecef, simulation_state.noisy_solution.vel_ecef);
}

void simulation_step_tracking_and_observations(double elapsed_seconds)
{
  (void)elapsed_seconds;

  double GPS_L1_LAMBDA = (GPS_C / GPS_L1_HZ);
  
  u8 week = -1; //TODO: calc week from day
  double t = simulation_state.noisy_solution.time.tow; //TODO: correct?

  //First we calculate all the current sat positions, velocities
  for (u8 i=0; i<simulation_num_almanacs; i++) {
    calc_sat_state_almanac(&simulation_almanacs[i], t, week, 
      simulation_sats_pos[i], simulation_sats_vel[i]);
  }


  //Calculate which sats are visible and select simulation_settings.num_sats of them
  //fill in selected_sats array
  u8 num_sats_selected = 0;
  double az, el;
  for (u8 i=0; i<simulation_num_almanacs; i++) {
    calc_sat_az_el_almanac(&simulation_almanacs[i], t, week,
                            simulation_state.true_pos_ecef, &az, &el);

    if (el > 0 && 
        num_sats_selected < simulation_settings.num_sats &&
        num_sats_selected < NAP_MAX_N_TRACK_CHANNELS) {      
      simulation_state.selected_sat_indices[num_sats_selected] = i;

      //Generate a code measurement which is just the pseudorange:
      double points_to_sat[3];
      vector_subtract(3, simulation_sats_pos[i], simulation_state.true_pos_ecef, points_to_sat);

      double distance_to_sat = vector_norm(3, points_to_sat);

      //Fill out the observation details into the NAV_MEAS structure for this satellite,
      //We simulate the pseudorange as a noisy range measurement, and
      //the carrier phase as a noisy range in wavelengths + an integer offset.
      navigation_measurement_t *nm = &simulation_state.nav_meas[num_sats_selected];
      nm->prn             = simulation_almanacs[i].prn;
      nm->raw_pseudorange = distance_to_sat + rand_gaussian(simulation_settings.pseudorange_variance);
      nm->carrier_phase   = distance_to_sat / GPS_L1_LAMBDA + simulation_fake_carrier_bias[i] + rand_gaussian(simulation_settings.carrier_phase_variance);
      nm->snr             = lerp(el, 0, M_PI/2, 4, 12) + rand_gaussian(simulation_settings.tracking_cn0_variance);      

      //As for tracking, we just set each sat consecutively in each channel.
      //This will cause weird jumps when a satellite rises or sets.
      simulation_state.tracking_channel[num_sats_selected].state = TRACKING_RUNNING;
      simulation_state.tracking_channel[num_sats_selected].prn = simulation_almanacs[i].prn;
      simulation_state.tracking_channel[num_sats_selected].cn0 = nm->snr;

      num_sats_selected++;
    }
  }

  simulation_state.noisy_solution.n_used = num_sats_selected;
  
}


/** Returns true if the simulation is at all enabled
*
*/

bool simulation_enabled(void) 
{
  return (simulation_settings.enabled > 0);
}

void sbp_send_simulation_enabled(void) 
{
  sbp_send_msg(MSG_SIMULATION_ENABLED, sizeof(uint8_t), &simulation_settings.enabled);
}

void sbp_send_simulation_settings(void) 
{
  sbp_send_msg(MSG_SIMULATION_SETTINGS, sizeof(simulation_settings), (u8 *) &simulation_settings);
}

/** Get current simulated PVT solution
* The structure returned by this changes every time simulation_step is called.
*/
inline gnss_solution* simulation_current_gnss_solution(void) 
{
  return &simulation_state.noisy_solution;
}

/** Get current simulated DOPS.
* The structure returned by this changes when settings are updated.
*/
inline dops_t* simulation_current_dops_solution(void) 
{
  return &simulation_state.dops;
}

/** Get current simulated baseline reference point.
* The structure returned by this changes when settings are updated.
*/
inline double* simulation_ref_ecef(void) 
{
  return simulation_settings.center_ecef;
}

/** Get current simulated baseline vector
* The structure returned by this changes every time simulation_step is called.
*/
inline double* simulation_baseline_ecef(void) 
{
  return simulation_state.true_baseline_ecef;
}

u8 simulation_current_num_sats(void)
{
  return simulation_state.noisy_solution.n_used;
}

tracking_state_msg_t simulator_get_tracking_state(u8 channel)
{
  if (channel >= simulation_current_num_sats()) {
    channel = simulation_current_num_sats() - 1;
  }
  return simulation_state.tracking_channel[channel];
}

navigation_measurement_t* simulator_get_navigation_measurements(void)
{
  return simulation_state.nav_meas;
}


/** Changes simulation mode when an SBP callback triggers this function
*
*/
void set_simulation_enabled_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;
  if (len == 1) {
    simulation_settings.enabled = (msg[0] != 0);
  }

  if (simulation_settings.enabled) {
    led_on(LED_RED);
  } else {
    led_off(LED_RED);
  }

  sbp_send_simulation_enabled();
  Notify("Simulation enabled: %d", simulation_settings.enabled);

}

/** Changes simulation mode when an SBP callback triggers this function
*
*/
void set_simulation_settings_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void) context;
  if (len == 0) {
    
    Notify("Sending current simulation settings.");
    sbp_send_simulation_settings();

  } else if (len == sizeof(simulation_settings)) {

    memcpy((uint8_t*)&simulation_settings, msg, len);
    //Clip values appropriately
    simulation_settings.num_sats = simulation_settings.num_sats;
    Notify("Received new simulation settings.");

  } else {

    Notify("Received malformed simulation settings: Incorrect size.");
  
  }
  
}

void simulator_setup_almanacs(void)
{

  //Do any setup we need for the satellite almanacs
  for (u8 i = 0; i < simulation_num_almanacs; i++) {
    simulation_fake_carrier_bias[i] = (rand() % 1000) * 10;    
  }

}
/** Must be called from main() or equivalent function before simulator runs
*
*/
void simulator_setup(void) 
{

  static sbp_msg_callbacks_node_t set_simulation_enabled_node;
  sbp_register_cbk(
    MSG_SIMULATION_ENABLED,
    &set_simulation_enabled_callback,
    &set_simulation_enabled_node
  );

  static sbp_msg_callbacks_node_t set_simulation_settings_node;
  sbp_register_cbk(
    MSG_SIMULATION_SETTINGS,
    &set_simulation_settings_callback,
    &set_simulation_settings_node
  );
  
  simulation_state.noisy_solution.time.wn = simulation_week_number;
  simulation_state.noisy_solution.time.tow = 0;

  simulator_setup_almanacs();
  
  sbp_send_simulation_settings();

}

/** \} */


