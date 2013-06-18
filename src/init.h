/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_INIT_H
#define SWIFTNAV_INIT_H

#include <libopencm3/stm32/f4/rcc.h>

extern const clock_scale_t hse_16_368MHz_in_65_472MHz_out_3v3;
extern const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3;
extern const clock_scale_t hse_16_368MHz_in_120_203MHz_out_3v3;

void init();

#endif

