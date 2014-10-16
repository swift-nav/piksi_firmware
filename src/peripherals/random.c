#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rng.h>
#include "random.h"

// These two methods copied from:
// https://raw.githubusercontent.com/libopencm3/libopencm3-examples/058298fd78c23639a1a135871e6c363c9d6153f6/examples/stm32/f4/stm32f4-discovery/random/random.c
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
uint32_t random_int(void)
{
  static uint32_t last_value;
  static uint32_t new_value;
  uint32_t error_bits = 0;
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

