/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_PPS_H
#define SWIFTNAV_PPS_H

#define PPS_WIDTH_MICROSECONDS 200000
#define PPS_NAP_CYCLES_OFFSET  -14
#define PPS_NAP_CLOCK_RATIO    8
#define PPS_THREAD_INTERVAL_MS 100

void pps_setup(void);

#endif
