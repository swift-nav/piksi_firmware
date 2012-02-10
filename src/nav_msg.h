#ifndef SWIFTNAV_NAV_MSG_H
#define SWIFTNAV_NAV_MSG_H

#include "int_types.h"

typedef struct {
  u32 subframe_bits[12];
  u16 subframe_bit_index;
  s16 subframe_preamble_index; // 0 = no preamble found, +x = preamble begins at bit index (x-1), -x = inverse preamble begins at (1-x)
  u8 bit_phase;
  u8 bit_phase_ref;
  u8 bit_phase_count;
  s32 nav_bit_integrate;
  
} nav_msg_t;


void nav_msg_init(nav_msg_t *n);
void nav_msg_update(nav_msg_t *n, s32 corr_prompt_real);

#endif

