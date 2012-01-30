#include "nav_msg.h"
#include <stdio.h>

#define NAV_MSG_BIT_PHASE_THRES 4

void nav_msg_init(nav_msg_t *n) {
  // Initialize the necessary parts of the nav message state structure
  n->subframe_bit_index = 0;
  n->bit_phase = 0;
  n->bit_phase_ref = 0;
  n->bit_phase_count = 0;
  n->nav_bit_integrate = 0;
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
      if (n->bit_phase_count > 2)
        printf("NAV_MSG: %d, %d", n->bit_phase_ref, n->bit_phase_count);
    }
    n->nav_bit_integrate = corr_prompt_real;  // Store the correlation for next time
 
  } else {
    // We have bit phase lock
    if (n->bit_phase == n->bit_phase_ref) {
      // Dump the nav bit, i.e. determine the sign of the correlation over the nav bit period
      if (n->nav_bit_integrate > 0) // Is bit 1?
        n->subframe_bits[n->subframe_bit_index >> 3] |= 1 << (n->subframe_bit_index & 0x07);
      else  // integrated correlation is negative, so bit is 0
        n->subframe_bits[n->subframe_bit_index >> 3] &= ~(1 << (n->subframe_bit_index & 0x07));
      n->subframe_bit_index++;

      if (n->subframe_bit_index == 300) {
        printf("NAV_MSG: Received a subframe: (look for 10001011 or 01110100)\n");
        for (int i = 0; i < 300; i++) {
          printf("%d", (n->subframe_bits[i >> 3] & (1 << (i & 0x07))) ? 1 : 0);
          if (i % 10 == 9) printf(" ");
          if (i % 80 == 79) printf("\n");
        }
        printf("\n Tracking is probably hosed now because of all that printing. EIT!\n");
      }

      n->nav_bit_integrate = 0; // Zero the integrator for the next nav bit
    }
    n->nav_bit_integrate += corr_prompt_real; // Sum the correlations over the 20 ms bit period
  }

}
