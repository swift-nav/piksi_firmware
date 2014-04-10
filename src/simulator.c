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

#include <libswiftnav/coord_system.h>
#include <ch.h>

#include "simulator.h"
#include "sbp_piksi.h"
#include "board/leds.h"
#include "sbp.h"

/** \simulator 
 * \{ */

simulation_settings_t simulation_settings = {
  .center_ecef = {-2700303.10144031,-4292474.39651309,3855434.34087421},
  .speed = 4.0,
  .radius = 100.0,
  .pos_variance = 2.0,
  .speed_variance = 0.02,
  .starting_week_number = 1768,
  .num_sats = 9,
  .mode = SIM_DISABLED,
};

simulation_state_t simulation_state = {
    .last_update_ticks = 0,
    .current_angle_rad = 0.0
};

gnss_solution simulation_solution; //No initialization

//Fill out fake dops
dops_t simulation_dops = {
  .pdop = 1.9,
  .gdop = 1.8,
  .tdop = 1.7,
  .hdop = 1.6,
  .vdop = 1.5,
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


/** Performs a timestep of the simulation that flies in a circle around a point.
* Updates the simulation_state and simulation_solution structs.
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
  simulation_solution.time.tow += 1000.0*elapsed_seconds;

  //Update the angle, making a small angle approximation.
  simulation_state.current_angle_rad += (simulation_settings.speed * elapsed_seconds) / simulation_settings.radius;
  if (simulation_state.current_angle_rad > 2*M_PI) {
    simulation_state.current_angle_rad = 0;
  }

  double pos_ned[3] = { 
    simulation_settings.radius * sin(simulation_state.current_angle_rad) + rand_gaussian(simulation_settings.pos_variance),
    simulation_settings.radius * cos(simulation_state.current_angle_rad) + rand_gaussian(simulation_settings.pos_variance), 
    rand_gaussian(simulation_settings.pos_variance)
  };

  //Fill out position simulation's gnss_solution pos_ECEF, pos_LLH structures
  wgsned2ecef_d(pos_ned, simulation_settings.center_ecef, simulation_solution.pos_ecef);
  wgsecef2llh(simulation_solution.pos_ecef, simulation_solution.pos_llh);

  //Calculate Velocity vector tangent to the sphere
  double noisy_speed = simulation_settings.speed + rand_gaussian(simulation_settings.speed_variance);

  simulation_solution.vel_ned[0] = noisy_speed * cos(simulation_state.current_angle_rad);
  simulation_solution.vel_ned[1] = noisy_speed * -1.0 * sin(simulation_state.current_angle_rad);
  simulation_solution.vel_ned[2] = 0.0;

  wgsned2ecef(simulation_solution.vel_ned, simulation_solution.pos_ecef, simulation_solution.vel_ecef);

}

/** Returns true if the simulation is at all enabled
*
*/

bool simulation_enabled(void) 
{
	return (simulation_settings.mode > SIM_DISABLED);
}

/** Returns true if at least this level of the simulation is enabled
*
*/
bool simulation_enabled_for(simulation_mode_t mode) 
{
	return (simulation_settings.mode >= mode);
}

void sbp_send_simulation_mode(void) 
{
  sbp_send_msg(MSG_SIMULATION_MODE, sizeof(simulation_mode_t), &simulation_settings.mode);
}

void sbp_send_simulation_settings(void) 
{
    sbp_send_msg(MSG_SIMULATION_SETTINGS, sizeof(simulation_settings), (u8 *) &simulation_settings);
}

inline gnss_solution* simulation_current_gnss_solution(void) {
	return &simulation_solution;
}

inline dops_t* simulation_current_dops_solution(void) {
	return &simulation_dops;
}

/** Changes simulation mode when an SBP callback triggers this function
*
*/
void set_simulation_mode_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;
  if (len == 1) {
  	simulation_settings.mode = msg[0];
  }

  if (simulation_settings.mode > 0) {
    led_on(LED_RED);
  } else {
    led_off(LED_RED);
  }

  sbp_send_simulation_mode();
  Notify("Simulation Mode: %d", simulation_settings.mode);

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
	  Notify("Received new simulation settings.");

  } else {

  	Notify("Received malformed simulation settings: Incorrect size.");
  
  }
  
}
/** Must be called from main() or equivalent function before simulator runs
*
*/
void simulator_setup(void) {

  //Setting up callback to listen for simulation being enabled or settings changed.
  static sbp_msg_callbacks_node_t set_simulation_mode_node;
  sbp_register_cbk(
    MSG_SIMULATION_MODE,
    &set_simulation_mode_callback,
    &set_simulation_mode_node
  );

  static sbp_msg_callbacks_node_t set_simulation_settings_node;
  sbp_register_cbk(
    MSG_SIMULATION_SETTINGS,
    &set_simulation_settings_callback,
    &set_simulation_settings_node
  );
  
  simulation_solution.time.wn = simulation_settings.starting_week_number;
  simulation_solution.time.tow = 0;
  simulation_solution.n_used = simulation_settings.num_sats;

  sbp_send_simulation_settings();

}

/** \} */


