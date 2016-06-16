/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_SID_SET_H_
#define SRC_SID_SET_H_

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

typedef struct {
  u64 sats[CODE_COUNT];
} gnss_sid_set_t;

void sid_set_init(gnss_sid_set_t *sid_set);
void sid_set_add(gnss_sid_set_t *sid_set, const gnss_signal_t sid);
u32 sid_set_get_sat_count(const gnss_sid_set_t *sid_set);

#endif /* SRC_SID_SET_H_ */
