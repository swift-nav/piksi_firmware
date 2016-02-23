/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libsbp/tracking.h>
#include <libswiftnav/common.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/track.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/bit_sync.h>

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup tracking
 * \{ */

#define TRACKING_ELEVATION_UNKNOWN 100 /* Ensure it will be above elev. mask */

/** \} */

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
s8 nav_bit_quantize(s32 bit_integrate);

bool tracking_channel_available(u8 channel, gnss_signal_t sid);
void tracking_channel_init(u8 channel, gnss_signal_t sid, float carrier_freq,
                           u32 start_sample_count, float cn0_init, s8 elevation);
bool tracking_channel_running(u8 channel);

void tracking_channels_update(u32 channels_mask);
void tracking_channel_disable(u8 channel);
void tracking_channel_ambiguity_unknown(u8 channel);
void tracking_update_measurement(u8 channel, channel_measurement_t *meas);
void tracking_send_state(void);
void tracking_setup(void);
void tracking_drop_satellite(gnss_signal_t sid);
bool tracking_channel_cn0_useable(u8 channel);
u32 tracking_channel_running_time_ms_get(u8 channel);
u32 tracking_channel_cn0_useable_ms_get(u8 channel);
u32 tracking_channel_cn0_drop_ms_get(u8 channel);
u32 tracking_channel_ld_opti_unlocked_ms_get(u8 channel);
u32 tracking_channel_ld_pess_locked_ms_get(u8 channel);
u32 tracking_channel_last_mode_change_ms_get(u8 channel);
gnss_signal_t tracking_channel_sid_get(u8 channel);
double tracking_channel_carrier_freq_get(u8 channel);
s32 tracking_channel_tow_ms_get(u8 channel);
bool tracking_channel_bit_sync_resolved(u8 channel);
bool tracking_channel_bit_polarity_resolved(u8 channel);
bool tracking_channel_evelation_degrees_set(gnss_signal_t sid, s8 elevation);
s8 tracking_channel_evelation_degrees_get(u8 channel);
bool tracking_channel_nav_bit_get(u8 channel, s8 *soft_bit);
bool tracking_channel_time_sync(u8 channel, s32 TOW_ms, s8 bit_polarity);

#endif
