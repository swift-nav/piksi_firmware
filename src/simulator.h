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

/** \addtogroup simulator
 * \{ */

/** \defgroup simulator GPS Simulator
 * Functions used to simulate PVT and baseline fixes for hardware-in-the-loop testing.
 * Generates GPS output from Piksi as if the GPS was performing solutions.
 *
 *   Currently only supports single-point-position messages.
 * \{ */

/* User-configurable GPS Simulator Settings */
typedef struct {
  u8      mode;                 //0 == off, 1 == PVT only, 2 == PVT & Baseline
  float   speed;                //speed (variance of velocity) in meters per second
  float   radius;               //radius of circle in meters
  float   pos_variance;         //in meters squared
  float   speed_variance;       //variance in speed (magnitude of velocity) in meters squared
  double  center_ecef[3];       //centerpoint that defines simulation absolute location
  u16     starting_week_number; //time start point
  u8      num_sats;             //number of simulated satellites to report
} simulation_settings_t;

/* Internal Simulation State */
typedef struct {
  u32     last_update_ticks;
  float   current_angle_rad;
} simulation_state_t;

/** \} */

//Simulation Input Parameters:
extern simulation_settings_t simulation_settings;

//Internal State:
extern simulation_state_t    simulation_state;

//Simulation Output:
extern gnss_solution         simulation_solution;
extern dops_t                simulation_dops;

//Math Helpers:
double rand_gaussian(const double variance);

//Running the Simulation:
void simulation_step(void);

//Initialization:
void set_simulation_settings_callback(u16 sender_id, u8 len, u8 msg[], void* context);
void simulator_setup(void);


#endif  /* SWIFTNAV_SIMULATOR_H */

