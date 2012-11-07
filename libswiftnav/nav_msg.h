/*
 * Copyright (C) 2010 Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTLIB_NAV_MSG_H
#define SWIFTLIB_NAV_MSG_H

#include "common.h"
#include "ephemeris.h"

#define NAV_MSG_SUBFRAME_BITS_LEN 12    // Buffer 384 nav bits

typedef struct {
  u32 subframe_bits[NAV_MSG_SUBFRAME_BITS_LEN];
  u16 subframe_bit_index;
  s16 subframe_start_index; // 0 = no preamble found, +x = preamble begins at bit index (x-1), -x = inverse preamble begins at (1-x)
  u8 bit_phase;
  u8 bit_phase_ref;
  u8 bit_phase_count;
  s32 nav_bit_integrate;

  u32 frame_words[3][8];
  u8 next_subframe_id;
} nav_msg_t;


void nav_msg_init(nav_msg_t *n);
u32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real);
bool subframe_ready(nav_msg_t *n);
void process_subframe(nav_msg_t *n, ephemeris_t *e);

#endif /* SWIFTLIB_NAV_MSG_H */

