#ifndef SWIFTNAV_NAV_MSG_H
#define SWIFTNAV_NAV_MSG_H

#include "int_types.h"
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
  
} nav_msg_t;


void nav_msg_init(nav_msg_t *n);
u32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real);
void process_subframe(nav_msg_t *n, ephemeris_t *e);

#endif

