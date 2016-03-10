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

#include <hal.h>
#include <ch.h>

#include "libswiftnav/common.h"
#include "libswiftnav/logging.h"
#include "random.h"


#define RANDOM_MAX_TRIES 100

static MUTEX_DECL(rng_mutex);

/* These two methods copied from:
 * https://raw.githubusercontent.com/libopencm3/libopencm3-examples/058298fd78c23639a1a135871e6c363c9d6153f6/examples/stm32/f4/stm32f4-discovery/random/random.c
 */
void rng_setup(void)
{
  rccEnableAHB2(RCC_AHB2ENR_RNGEN, TRUE);
  /* Enable interupt */
  /* Set the IE bit in the RNG->CR register. */
  RNG->CR |= RNG_CR_IE;
  /* Enable the random number generation by setting the RNGEN bit in
   * the RNG_CR register. This activates the analog part, the RNG_LFSR
   * and the error detector.
   * */
  RNG->CR |= RNG_CR_RNGEN;
}

u32 random_int(void)
{
  chMtxLock(&rng_mutex);

  static u32 last_value;
  u32 new_value = last_value;
  int tries = 0;
  do {
    u32 sr;
    /* Check for error flags. */
    if ((sr = RNG->SR)) {
      if (sr & RNG_SR_SEIS) {
        /* Seed error, we must reinitialise */
        log_warn("random: Seed error, restarting RNG 0x%"PRIx32"", sr);
        RNG->CR &= ~RNG_CR_RNGEN;
        RNG->CR |= RNG_CR_RNGEN;
      }
      /* Write back as zero to clear */
      RNG->SR = ~(sr & (RNG_SR_SEIS | RNG_SR_CEIS));
    }
    /* Check if data is ready. */
    if ((RNG->SR & RNG_SR_DRDY) == 0)
      continue;

    tries++;
    new_value = RNG->DR;
  } while ((new_value == last_value) && (tries < RANDOM_MAX_TRIES));

  if (new_value == last_value) {
    log_error("random: Gave up waiting for random number!");
    new_value = last_value + 1;
  }
  last_value = new_value;

  chMtxUnlock(&rng_mutex);
  return new_value;
}
