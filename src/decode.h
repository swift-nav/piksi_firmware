/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_H
#define SWIFTNAV_DECODE_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

void decode_setup(void);

bool decoder_channel_available(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_disable(u8 tracking_channel);

#endif
