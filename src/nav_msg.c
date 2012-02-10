#include "nav_msg.h"
#include <stdio.h>
#include <string.h>

#define NAV_MSG_BIT_PHASE_THRES 5

void nav_msg_init(nav_msg_t *n) {
  // Initialize the necessary parts of the nav message state structure
  n->subframe_bit_index = 0;
  n->bit_phase = 0;
  n->bit_phase_ref = 0;
  n->bit_phase_count = 0;
  n->nav_bit_integrate = 0;
  n->subframe_preamble_index = 0;
  memset(n->subframe_bits,0,sizeof(n->subframe_bits));

}

void nav_msg_update(nav_msg_t *n, s32 corr_prompt_real) {
  // Called once per tracking loop update (atm fixed at 1 PRN [1 ms])
  // Performs the necessary steps to recover the nav bit clock, store the nav bits
  // and decode them.
  
  // Do we have bit phase lock yet? (Do we know which of the 20 possible PRN offsets corresponds to the nav bit edges?)
    n->bit_phase++;
    n->bit_phase %= 20;

  if (n->bit_phase_count < NAV_MSG_BIT_PHASE_THRES) {
    // No bit phase lock yet
    if ((n->nav_bit_integrate > 0) != (corr_prompt_real > 0)) {
      // Edge detected
      if (n->bit_phase == n->bit_phase_ref)
        n->bit_phase_count++; // This edge came N*20 ms after the last one
      else {
        n->bit_phase_ref = n->bit_phase;  // Store the bit phase hypothesis
        n->bit_phase_count = 1;
      }
    }
    n->nav_bit_integrate = corr_prompt_real;  // Store the correlation for next time
 
  } else {
    // We have bit phase lock
    if (n->bit_phase == n->bit_phase_ref) {
      // Dump the nav bit, i.e. determine the sign of the correlation over the nav bit period
      if (n->nav_bit_integrate > 0) // Is bit 1?
        n->subframe_bits[n->subframe_bit_index >> 5] |=   1 << (31 - (n->subframe_bit_index & 0x1F));
      else  // integrated correlation is negative, so bit is 0
        n->subframe_bits[n->subframe_bit_index >> 5] &= ~(1 << (31 - (n->subframe_bit_index & 0x1F)));

      n->nav_bit_integrate = 0; // Zero the integrator for the next nav bit
      
      n->subframe_bit_index++;
      if (n->subframe_bit_index == 12*32)  n->subframe_bit_index = 0;
      
      u8 bix_hi = n->subframe_bit_index >> 5;
      u8 bix_lo = n->subframe_bit_index & 0x1F;

      // Check whether there's a preamble at the start of the circular subframe_bits buffer
      u32 preamble_candidate = n->subframe_bits[bix_hi] << bix_lo;
      if (bix_lo)
        preamble_candidate |= n->subframe_bits[(bix_hi + 1) % (12*32)] >> (32 - bix_lo);
      

      if (preamble_candidate >> 24 == 0x8B) {
        printf("NAV_MSG: Found preamble : %08X\n",(unsigned int)preamble_candidate);
       // n->subframe_preamble_index = n->subframe_;
      }
      if (preamble_candidate >> 24 == 0x74) {
        printf("NAV_MSG: Found ~preamble: %08X\n",(unsigned int)~preamble_candidate);
       // n->subframe_preamble_index = n->subframe_;
       // n->subframe_status = NAV_BITS_SUBFRAME_INVERSE_PREAMBLE;
      }
    }
    n->nav_bit_integrate += corr_prompt_real; // Sum the correlations over the 20 ms bit period
  }

}
