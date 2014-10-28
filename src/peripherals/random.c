/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Scott Kovach <scott@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rng.h>

#include "libswiftnav/common.h"
#include "random.h"

/* These two methods copied from:
 * https://raw.githubusercontent.com/libopencm3/libopencm3-examples/058298fd78c23639a1a135871e6c363c9d6153f6/examples/stm32/f4/stm32f4-discovery/random/random.c
 */
void rng_setup(void)
{
  rcc_periph_clock_enable(RCC_RNG);
  /* Enable interupt */
  /* Set the IE bit in the RNG_CR register. */
  RNG_CR |= RNG_CR_IE;
  /* Enable the random number generation by setting the RNGEN bit in
   * the RNG_CR register. This activates the analog part, the RNG_LFSR
   * and the error detector.
   * */
  RNG_CR |= RNG_CR_RNGEN;
}

u32 random_int(void)
{
  static u32 last_value;
  static u32 new_value;
  u32 error_bits = 0;
  error_bits = RNG_SR_SEIS | RNG_SR_CEIS;
  while (new_value == last_value) {
    /* Check for error flags and if data is ready. */
    if (((RNG_SR & error_bits) == 0) &&
        ((RNG_SR & RNG_SR_DRDY) == 1)) {
      new_value = RNG_DR;
    }
  }
  last_value = new_value;
  return new_value;
}
