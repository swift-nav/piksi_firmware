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

#ifndef SWIFTNAV_SIMULATOR_H
#define SWIFTNAV_SIMULATOR_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include "track.h"

/** \addtogroup simulator
 * \{ */

/** \defgroup simulator GPS Simulator
 * Functions used to simulate PVT and baseline fixes for hardware-in-the-loop testing.
 * Generates GPS output from Piksi as if the GPS was performing solutions.
 *
 *   Currently only supports single-point-position messages.
 * \{ */

typedef uint8_t simulation_mode_t; /* Force uint8_t size for simulation_mode */

/* User-configurable GPS Simulator Settings 
 * WARNING: THIS STRUCT IS PACKED! CAREFUL MEMORY ALIGNMENT!
*/
typedef struct __attribute__((packed)) { 
  double            center_ecef[3];        /**< centerpoint that defines simulation absolute location */
  float             speed;                 /**< speed (variance of velocity) in meters per second */
  float             radius;                /**< radius of circle in meters */
  float             pos_variance;          /**< in meters squared */
  float             speed_variance;        /**< variance in speed (magnitude of velocity) in meters squared */
  float             tracking_cn0_variance; /**< variance in signal-to-noise ratio of tracking channels */
  float             pseudorange_variance;  /**< variance in each sat's simulated pseudorange */
  float             carrier_phase_variance;/**< variance in each sat's simulated carrier phase */
  u8                num_sats;              /**< number of simulated satellites to report */
  u8                enabled;               /**< Current mode of simulation */
} simulation_settings_t;

/* Internal Simulation State */
typedef struct {
  u32            last_update_ticks;
  float          current_angle_rad;
  double         true_pos_ecef[3];
  double         true_baseline_ecef[3];

  //Accessing the simulator_data.c almanacs:
  u8             num_sats_selected;
  u8             selected_sat_indices[NAP_MAX_N_TRACK_CHANNELS];

  //Simulated solutions
  tracking_state_msg_t      tracking_channel[NAP_MAX_N_TRACK_CHANNELS];
  navigation_measurement_t  nav_meas[NAP_MAX_N_TRACK_CHANNELS];
  dops_t                    dops;
  gnss_solution             noisy_solution;

} simulation_state_t;

/** \} */

//Math Helpers:
double rand_gaussian(const double variance);
double lerp(double t, double u, double v, double x, double y);

//Running the Simulation:
void simulation_step(void);
bool simulation_enabled();

//Internals of the simulator
void simulation_step_position_in_circle(double);
void simulation_step_tracking_and_observations(double);

//Sending simulation settings to the outside world
void sbp_send_simulation_mode(void);
void sbp_send_simulation_settings(void);

//Getting data from the simulation
gnss_solution* simulation_current_gnss_solution(void);
dops_t* simulation_current_dops_solution(void);
double* simulation_ref_ecef(void);
double* simulation_baseline_ecef(void);
u8 simulation_current_num_sats(void);
tracking_state_msg_t simulator_get_tracking_state(u8 channel);
navigation_measurement_t* simulator_get_navigation_measurements(void);

//Initialization:
void set_simulation_settings_callback(u16 sender_id, u8 len, u8 msg[], void* context);
void simulator_setup_almanacs(void);
void simulator_setup(void);


#endif  /* SWIFTNAV_SIMULATOR_H */

