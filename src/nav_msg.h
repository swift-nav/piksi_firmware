#ifndef SWIFTNAV_NAV_MSG_H
#define SWIFTNAV_NAV_MSG_H

#include "int_types.h"

#define NAV_BITS_SUBFRAME_INVALID           0
#define NAV_BITS_SUBFRAME_PREAMBLE          1
#define NAV_BITS_SUBFRAME_INVERSE_PREAMBLE  2


typedef struct {
  u8 subframe_bits[38];
  u16 subframe_bit_index;
  u8 subframe_status;
  u8 bit_phase;
  u8 bit_phase_ref;
  u8 bit_phase_count;
  s32 nav_bit_integrate;
  
} nav_msg_t;


void nav_msg_init(nav_msg_t *n);
void nav_msg_update(nav_msg_t *n, s32 corr_prompt_real);

#endif

