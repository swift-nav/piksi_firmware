/*
 * Copyright (C) 2012-2013 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_CW_H
#define SWIFTNAV_CW_H

#include <libswiftnav/common.h>

/** \addtogroup cw
 * \{ */

#define SPECTRUM_LEN 301

/** Status of SwiftNAP CW channel. */
typedef enum {
  CW_DISABLED = 0,
  CW_LOADING,
  CW_LOADING_DONE,
  CW_RUNNING,
  CW_RUNNING_DONE
} cw_status_t;

/** CW search state. */
typedef struct {
  cw_status_t state; /**< Status of SwiftNAP CW channel. */
  s32 freq_step;     /**< Step size between interference freq search points. */
  s32 freq_min;      /**< Lowest interference freq to search. */
  s32 freq_max;      /**< Highest interference freq to search. */
  s32 freq;          /**< Interference freq of next correlation to be read. */
	u16 count;         /**< Total number of interference freq points searched. */
	u64 spectrum_power[SPECTRUM_LEN]; /**< Array of power. */
} cw_state_t;

/** Struct sent by host to cw_start_callback to start a CW search. */
typedef struct {
  float freq_min;  /**< Lowest interference freq to search. */
  float freq_max;  /**< Highest interference freq to search. */
  float freq_step; /**< Step size between interference freq search points. */
} cw_start_msg_t;

/** \} */

void cw_schedule_load(u32 count);
void cw_service_load_done();
u8 cw_get_load_done();
u8 cw_get_running_done();

void cw_setup();
void cw_start(float freq_min, float freq_max, float freq_bin_width);
void cw_service_irq();
void cw_send_result(float freq, u64 power);
void cw_get_spectrum_point(float* freq, u64* power, u16 index);

#endif
