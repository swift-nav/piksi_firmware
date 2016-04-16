/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_HW_V3_H
#define SWIFTNAV_NAP_HW_V3_H

#define FPGA_FREQ 100e6
#define SAMPLE_FREQ 99.375e6
#define TRACK_SAMPLE_FREQ (SAMPLE_FREQ / 4)

#include "nap/nap_regs.h"
#include <gic.h>

/* NAP track channel parameters. */
#define NAP_TRACK_CARRIER_FREQ_WIDTH              32
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  (((u64)1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)TRACK_SAMPLE_FREQ)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP / TRACK_SAMPLE_FREQ)

#define IRQ_ID_NAP_TRACK IRQ_ID_FPGA3
#define NAP_IRQ_PRIORITY 4

#endif

