#include "nav_msg.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


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
  n->next_subframe_id = 1;

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
              // We got two appropriately spaced preambles, and two matching TOW counts.  Pretty certain now.
            
              // The TOW in the message is for the start of the NEXT subframe.
              // That is, 240 nav bits' time from now, since we are 60 nav bits into the second subframe that we recorded. 
              if (TOW_trunc)
                TOW_ms = TOW_trunc * 6000 - (300-60)*20;
              else  // end of week special case
                TOW_ms = 7*24*60*60*1000 - (300-60)*20; 
              //printf("TOW = hh:%02d:%02d.%03d\n", (int) (TOW_ms / 60000 % 60), (int)(TOW_ms / 1000 % 60), (int)(TOW_ms % 1000));
              
            } else
              n->subframe_start_index = 0;  // the TOW counts didn't match - disregard.
          } else
            n->subframe_start_index = 0;    // didn't find a second preamble in the right spot - disregard.
        }
      }
    }
  }
  return TOW_ms;
}


int parity(u32 x) {
  // Returns 1 if there are an odd number of bits set
  x ^= x >> 1;
  x ^= x >> 2;
  x ^= x >> 4;
  x ^= x >> 8;
  x ^= x >> 16;
  return (x & 1);
}

int nav_parity(u32 *word) {
// expects a word where MSB = D29*, bit 30 = D30*, bit 29 = D1, ... LSB = D30 as described in IS-GPS-200E Table 20-XIV
// Inverts the bits if necessary, and checks the parity.
// Returns 0 for success, 1 for fail.

  if (*word & 1<<30)     // inspect D30*
    *word ^= 0x3FFFFFC0; // invert all the data bits!

 // printf("w=%08X  ",(unsigned int )word);

  if (parity(*word & 0b10111011000111110011010010100000)) // check d25 (see IS-GPS-200E Table 20-XIV)
    return 25;

  if (parity(*word & 0b01011101100011111001101001010000)) // check d26
    return 26;

  if (parity(*word & 0b10101110110001111100110100001000)) // check d27
    return 27;

  if (parity(*word & 0b01010111011000111110011010000100)) // check d28
    return 28;

  if (parity(*word & 0b01101011101100011111001101000010)) // check d29
    return 29;

  if (parity(*word & 0b10001011011110101000100111000001)) // check d30
    return 30;

  return 0;
}




void process_subframe(nav_msg_t *n, ephemeris_t *e) {
  // Check parity and parse out the ephemeris from the most recently received subframe
  
  // First things first - check the parity, and invert bits if necessary.
  // process the data, skipping the first word, TLM, and starting with HOW

  // printf("  %d  ", (n->subframe_start_index > 0));

  if (!e) {
    printf(" process_subframe: CALLED WITH e = NULL!\n");
    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id = 1;      // Make sure we start again next time
    return;
  }
  u32 sf_word2 = extract_word(n, 28, 32, 0);
  if (nav_parity(&sf_word2)) {
      printf("SUBFRAME PARITY ERROR (word 2)\n");
      n->subframe_start_index = 0;  // Mark the subframe as processed
      n->next_subframe_id = 1;      // Make sure we start again next time
      return;
  }

  u8 sf_id = sf_word2 >> 8 & 0x07;    // Which of 5 possible subframes is it?

  printf("sf_id = %d\n",sf_id);

  if (sf_id <= 3 && sf_id == n->next_subframe_id) {  // Is it the one that we want next?
    
    for (int w = 0; w < 8; w++) {   // For words 3..10
      n->frame_words[sf_id-1][w] = extract_word(n, 30*(w+2) - 2, 32, 0);    // Get the bits
      // MSBs are D29* and D30*.  LSBs are D1...D30
      if (nav_parity(&n->frame_words[sf_id-1][w])) {  // Check parity and invert bits if D30*
        printf("SUBFRAME PARITY ERROR (word %d)\n", w+3);
        n->next_subframe_id = 1;      // Make sure we start again next time
        n->subframe_start_index = 0;  // Mark the subframe as processed
        return;
      }      
    }
    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id++;

    if (sf_id == 3) {
      // Got all of subframes 1 to 3
      n->next_subframe_id = 1;      // Make sure we start again next time

      // Now let's actually go through the parameters...
      
      // These unions facilitate signed/unsigned conversion and sign extension
      union {
        char s8;
        unsigned char u8;
      } onebyte;

      union
      {
        short s16;
        unsigned short u16;
      } twobyte;

      union
      {
        int s32;
        unsigned u32;
      } fourbyte;

      // Subframe 1: SV health, T_GD, t_oc, a_f2, a_f1, a_f0
      
      e->healthy = !(n->frame_words[0][3-3] >> (30-17) & 1);     // Health flag: Word 3, bit 17
      if (!e->healthy)
        printf("UNHEALTHY");
     
      onebyte.u8 = n->frame_words[0][7-3] >> (30-24) & 0xFF;  // t_gd: Word 7, bits 17-24
      e->tgd = onebyte.s8 * pow(2,-31);

      e->toc = (n->frame_words[0][8-3] >> (30-24) & 0xFFFF) * 16;   // t_oc: Word 8, bits 8-24

      onebyte.u8 = n->frame_words[0][9-3] >> (30-8) & 0xFF;         // a_f2: Word 9, bits 1-8
      e->af2 = onebyte.s8 * pow(2,-55);

      twobyte.u16 = n->frame_words[0][9-3] >> (30-24) & 0xFFFF;     // a_f1: Word 9, bits 9-24    
      e->af1 = twobyte.s16 * pow(2,-43);

      fourbyte.u32 = n->frame_words[0][10-3] >> (30-22) & 0x3FFFFF; // a_f0: Word 10, bits 1-22
      fourbyte.u32 <<= 10; // Shift to the left for sign extension             
      fourbyte.s32 >>= 10; // Carry the sign bit back down and reduce to signed 22 bit value
      e->af0 = fourbyte.s32 * pow(2,-31);


      // Subframe 2: crs, dn, m0, cuc, ecc, cus, sqrta, toe
      
      twobyte.u16 = n->frame_words[1][3-3] >> (30-24) & 0xFFFF;     // crs: Word 3, bits 9-24
      e->crs = twobyte.s16 * pow(2,-5);

      twobyte.u16 = n->frame_words[1][4-3] >> (30-16) & 0xFFFF;     // dn: Word 4, bits 1-16
      e->dn = twobyte.s16 * pow(2,-43) * M_PI;

      fourbyte.u32 = ((n->frame_words[1][4-3] >> (30-24) & 0xFF) << 24) // m0: Word 4, bits 17-24              
                  | (n->frame_words[1][5-3] >> (30-24) & 0xFFFFFF);     // and word 5, bits 1-24 
      e->m0 = fourbyte.s32 * pow(2,-31) * M_PI;

      twobyte.u16 = n->frame_words[1][6-3] >> (30-16) & 0xFFFF;    // cuc: Word 6, bits 1-16
      e->cuc = twobyte.s16 * pow(2,-29);

      fourbyte.u32 = ((n->frame_words[1][6-3] >> (30-24) & 0xFF) << 24) // ecc: Word 6, bits 17-24              
                  | (n->frame_words[1][7-3] >> (30-24) & 0xFFFFFF);     // and word 7, bits 1-24 
      e->ecc = fourbyte.u32 * pow(2,-33);

      
      twobyte.u16 = n->frame_words[1][8-3] >> (30-16) & 0xFFFF;   // cus: Word 8, bits 1-16
      e->cus = twobyte.s16 * pow(2,-29);


      fourbyte.u32 = ((n->frame_words[1][8-3] >> (30-24) & 0xFF) << 24) // sqrta: Word 8, bits 17-24              
                  | (n->frame_words[1][9-3] >> (30-24) & 0xFFFFFF);     // and word 9, bits 1-24 
      e->sqrta = fourbyte.u32 * pow(2,-19);

      e->toe = (n->frame_words[1][10-3] >> (30-16) & 0xFFFF) * 16;   // t_oe: Word 10, bits 1-16


      // Subframe 3: cic, omega0, cis, inc, crc, w, omegadot, inc_dot

      twobyte.u16 = n->frame_words[2][3-3] >> (30-16) & 0xFFFF;   // cic: Word 3, bits 1-16
      e->cic = twobyte.s16 * pow(2,-29);
      
      fourbyte.u32 = ((n->frame_words[2][3-3] >> (30-24) & 0xFF) << 24) // omega0: Word 3, bits 17-24              
                  | (n->frame_words[2][4-3] >> (30-24) & 0xFFFFFF);     // and word 4, bits 1-24 
      e->omega0 = fourbyte.s32 * pow(2,-31) * M_PI; 

      twobyte.u16 = n->frame_words[2][5-3] >> (30-16) & 0xFFFF; // cis: Word 5, bits 1-16
      e->cis = twobyte.s16 * pow(2,-29);

      fourbyte.u32 = ((n->frame_words[2][5-3] >> (30-24) & 0xFF) << 24) // inc (i0): Word 5, bits 17-24              
                  | (n->frame_words[2][6-3] >> (30-24) & 0xFFFFFF);     // and word 6, bits 1-24 
      e->inc = fourbyte.s32 * pow(2,-31) * M_PI; 

      twobyte.u16 = n->frame_words[2][7-3] >> (30-16) & 0xFFFF; // crc: Word 7, bits 1-16
      e->crc = twobyte.s16 * pow(2,-5);

      fourbyte.u32 = ((n->frame_words[2][7-3] >> (30-24) & 0xFF) << 24) // w (omega): Word 7, bits 17-24              
                  | (n->frame_words[2][8-3] >> (30-24) & 0xFFFFFF);     // and word 8, bits 1-24 
      e->w = fourbyte.s32 * pow(2,-31) * M_PI; 

      fourbyte.u32 = n->frame_words[2][9-3] >> (30-24) & 0xFFFFFF;     // Omega_dot: Word 9, bits 1-24
      fourbyte.u32 <<= 8; // shift left for sign extension
      fourbyte.s32 >>= 8; // sign-extend it
      e->omegadot = fourbyte.s32 * pow(2,-43) * M_PI;

      twobyte.u16 = n->frame_words[2][10-3] >> (30-22) & 0x3FFF;  // inc_dot (IDOT): Word 10, bits 9-22 
      twobyte.u16 <<= 2;
      twobyte.s16 >>= 2;  // sign-extend
      e->inc_dot = twobyte.s16 * pow(2,-43) * M_PI;

  
      e->valid = 1;

      /*printf("Health %d\n", e->healthy);*/
      /*printf("TGD %16g\n", e->tgd);*/
      /*printf("TOC %16u\n", (unsigned int)e->toc);*/
      /*printf("af2 %16g\n", e->af2);*/
      /*printf("af1 %16g\n", e->af1);*/
      /*printf("af0 %16g\n", e->af0);*/
      /*printf("CRS %16g\n", e->crs);*/
      /*printf("DN %16g\n", e->dn);*/
      /*printf("M0 %16g\n", e->m0);*/
      /*printf("CUC %16g\n", e->cuc);*/
      /*printf("Ecc %16g\n", e->ecc);*/
      /*printf("CUS %16g\n", e->cus);*/
      /*printf("SQRT A %16g\n", e->sqrta);*/
      /*printf("TOE %16u\n", (unsigned int)e->toe);*/
      /*printf("CIC %16g\n", e->cic);*/
      /*printf("omega0 %16g\n", e->omega0);*/
      /*printf("CIS %16g\n", e->cis);*/
      /*printf("Inc %16g\n", e->inc);*/
      /*printf("CRC %16g\n", e->crc);*/
      /*printf("W %16g\n", e->w);*/
      /*printf("omegadot %16g\n", e->omegadot);*/
      /*printf("inc_dot %16g\n", e->inc_dot);*/

    }
  } else {  // didn't get the subframe that we want next
      n->next_subframe_id = 1;      // Make sure we start again next time
      n->subframe_start_index = 0;  // Mark the subframe as processed
  }

        
}

