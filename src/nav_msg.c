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
  n->subframe_start_index = 0;
  memset(n->subframe_bits,0,sizeof(n->subframe_bits));

}

u32 extract_word(nav_msg_t *n, u16 bit_index, u8 n_bits, u8 invert) {
// Extract a word of n_bits length (n_bits <= 32) at position bit_index into the subframe
// Takes account of the offset stored in n, and the circular nature of the n->subframe_bits buffer

  if (n->subframe_start_index) {  // offset for the start of the subframe in the buffer
    if (n->subframe_start_index > 0)
      bit_index += n->subframe_start_index; // standard
    else { 
      bit_index -= n->subframe_start_index; // bits are inverse!
      invert = !invert;
    }
    
    bit_index--;
  }

  if (bit_index > NAV_MSG_SUBFRAME_BITS_LEN*32) // wrap if necessary
    bit_index -= NAV_MSG_SUBFRAME_BITS_LEN*32;

  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  u32 word = n->subframe_bits[bix_hi] << bix_lo;
  if (bix_lo) {
    bix_hi++;
    if (bix_hi == NAV_MSG_SUBFRAME_BITS_LEN)
      bix_hi=0;
    word |=  n->subframe_bits[bix_hi] >> (32 - bix_lo);
  }


  if (invert)
    word = ~word;
  
  return word >> (32 - n_bits);
}


u32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real) {
  // Called once per tracking loop update (atm fixed at 1 PRN [1 ms])
  // Performs the necessary steps to recover the nav bit clock, store the nav bits
  // and decode them.
  
  u32 TOW_ms = 0;

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
    if (n->bit_phase != n->bit_phase_ref)  
      n->nav_bit_integrate += corr_prompt_real; // Sum the correlations over the 20 ms bit period
    else {
      // Dump the nav bit, i.e. determine the sign of the correlation over the nav bit period
      if (n->nav_bit_integrate > 0) // Is bit 1?
        n->subframe_bits[n->subframe_bit_index >> 5] |=   1 << (31 - (n->subframe_bit_index & 0x1F));
      else  // integrated correlation is negative, so bit is 0
        n->subframe_bits[n->subframe_bit_index >> 5] &= ~(1 << (31 - (n->subframe_bit_index & 0x1F)));

      n->nav_bit_integrate = 0; // Zero the integrator for the next nav bit
      
      n->subframe_bit_index++;
      if (n->subframe_bit_index == NAV_MSG_SUBFRAME_BITS_LEN*32)
        n->subframe_bit_index = 0;
   
      // Yo dawg, are we still looking for the preamble?
      if (!n->subframe_start_index) {
 
        
        // We're going to look for the preamble at a time 360 nav bits ago, then again 60 nav bits ago.
        //
        #define SUBFRAME_START_BUFFER_OFFSET (NAV_MSG_SUBFRAME_BITS_LEN*32 - 360)
        
        // Check whether there's a preamble at the start of the circular subframe_bits buffer
        u8 preamble_candidate = extract_word(n, n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET, 8, 0);
      
        if (preamble_candidate == 0x8B) {
           n->subframe_start_index = n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1;
        }
        else if (preamble_candidate == 0x74) {
           n->subframe_start_index = -(n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1);
        }
        
        if (n->subframe_start_index) {
          // Looks like we found a preamble, but let's confirm.
          if (extract_word(n, 300, 8, 0) == 0x8B) {
            // There's another preamble in the following subframe.  Looks good so far.
            // Extract the TOW:
            
            unsigned int TOW_trunc = extract_word(n,30,17,extract_word(n,29,1,0)); // bit 29 is D30* for the second word, where the TOW resides.
            TOW_trunc++;  // Increment it, to see what we expect at the start of the next subframe
            if (TOW_trunc >= 7*24*60*10)  // Handle end of week rollover
              TOW_trunc = 0;

            if (TOW_trunc == extract_word(n,330,17,extract_word(n,329,1,0))) {
              // We got the TOW.

              // The TOW in the message is for the start of the NEXT subframe.
              // That is, 240 nav bits' time from now, since we are 60 nav bits into the second subframe that we recorded. 
              if (TOW_trunc)
                TOW_ms = TOW_trunc * 6000 - (300-60)*20;
              else  // end of week special case
                TOW_ms = 7*24*60*60*1000 - (300-60)*20; 
              //printf("TOW = hh:%02d:%02d.%03d\n", (int) (TOW_ms / 60000 % 60), (int)(TOW_ms / 1000 % 60), (int)(TOW_ms % 1000));
              
            }
          }
        }
      }
    }
  }
  return TOW_ms;
}

int nav_parity(u32 word) {
// expects a word where MSB = D29*, bit 30 = D30*, bit 29 = D1, ... LSB = D30 as described in IS-GPS-200E Table 20-XIV
// Inverts the bits if necessary, and checks the parity.
// Returns 0 for success, 1 for fail.

  if (word & 1<<30)     // inspect D30*
    word ^= 0x3FFFFFC0; // invert all the data bits!

  u32 p;

  #define NAV_PARITY_CHECK(mask) \
  p = word & mask; \
  p ^= p >> 1; p ^= p >> 2; p ^= p >> 4; p ^= p >> 8; p ^= p >> 16; \
  if (p) return 1;

  NAV_PARITY_CHECK(0b10111011000111110011010010100000); // check d25
  printf("d25 ok\n");

  NAV_PARITY_CHECK(0b01011101100011111001101001010000); // check d26
  printf("d26 ok\n");

  NAV_PARITY_CHECK(0b10101110110001111100110100001000); // check d27
  printf("d27 ok\n");

  NAV_PARITY_CHECK(0b01010111011000111110011010000100); // check d28
  printf("d28 ok\n");

  NAV_PARITY_CHECK(0b01101011101100011111001101000010); // check d29
  printf("d29 ok\n");

  NAV_PARITY_CHECK(0b10001011011110101000100111000001); // check d30
  printf("d30 ok\n");

  return 0;
}




void process_subframe(nav_msg_t *n, ephemeris_t *e __attribute__((unused))) {
  // Check parity and parse out the ephemeris from the most recently received subframe
  
  // First things first - check the parity, and invert bits if necessary.
  // process the data, skipping the first word, TLM, and starting with HOW

  u32 subframe_words[9];
  for (int i = 0; i < 9; i++) {
    subframe_words[i] = extract_word(n, 30+i, 32, 0);
    // MSBs are D29* and D30*.  LSBs are D1...D30
    if (nav_parity(subframe_words[i]))
      printf("process_subframe: Parity error!\n");
  }

  n->subframe_start_index = 0;  // Mark the subframe as processed

}

