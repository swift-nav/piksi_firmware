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
* Functions used to simulate PVT and baseline fixes for hardware-in-the-loop
* testing. Generates GPS output from Piksi as if the GPS was performing
* solutions.
*
* Expected usage:
* First call `simulation_setup()`,
* To update the simulation, call `simulation_step()`
* and to get the current simulated data, call any of the
* `simulation_current_xxx` functions
*
 * \{ */

#define SIM_PRN_OFFSET 200

typedef uint8_t simulation_mode_t; /* Force uint8_t size for simulation_mode */

typedef enum {
  SIMULATION_MODE_PVT      = (1<<0),
  SIMULATION_MODE_TRACKING = (1<<1),
  SIMULATION_MODE_FLOAT    = (1<<2),
  SIMULATION_MODE_RTK      = (1<<3)
} simulation_modes_t;

/* User-configurable GPS Simulator Settings
 * WARNING: THIS STRUCT IS PACKED! CAREFUL MEMORY ALIGNMENT!
*/
typedef struct __attribute__((packed)) {
  double            base_ecef[3];        /**< centerpoint that defines simulation absolute location */
  float             speed;                 /**< speed (variance of velocity) in meters per second */
  float             radius;                /**< radius of circle in meters */
  float             pos_sigma;          /**< in meters squared */
  float             speed_sigma;        /**< variance in speed (magnitude of velocity) in meters squared */
  float             cn0_sigma; /**< variance in signal-to-noise ratio of tracking channels */
  float             pseudorange_sigma;  /**< variance in each sat's simulated pseudorange */
  float             phase_sigma;/**< variance in each sat's simulated carrier phase */
  u8                num_sats;              /**< number of simulated satellites to report */
  u8                mode_mask;             /** < Current mode of the simulator */
} simulation_settings_t;

/** \} */

//Math Helpers:
double rand_gaussian(const double variance);
double lerp(double t, double u, double v, double x, double y);

//Running the Simulation:
void simulation_step(void);
bool simulation_enabled();
bool simulation_enabled_for(simulation_modes_t mode_mask);

//Internals of the simulator
void simulation_step_position_in_circle(double);
void simulation_step_tracking_and_observations(double);
void populate_nav_meas(navigation_measurement_t *, double, double, int);

//Sending simulation settings to the outside world
void sbp_send_simulation_enabled(void);

//Getting data from the simulation
gnss_solution*             simulation_current_gnss_solution(void);
dops_t*                    simulation_current_dops_solution(void);
double*                    simulation_ref_ecef(void);
double*                    simulation_current_baseline_ecef(void);
u8                         simulation_current_num_sats(void);
tracking_channel_state_t   simulation_current_tracking_state(u8 channel);
navigation_measurement_t*  simulation_current_navigation_measurements(void);

//Initialization:
void simulator_setup_almanacs(void);
void simulator_setup(void);


#endif  /* SWIFTNAV_SIMULATOR_H */

