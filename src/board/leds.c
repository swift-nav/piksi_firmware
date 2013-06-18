/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "leds.h"

/** \defgroup board Board
 * Interfaces for interacting with the rest of the Piksi hardware.
 * \{ */

/** \defgroup leds LEDs
 * Interface to light the LEDs.
 * \{ */

/** Setup the LED GPIOs. */
void led_setup(void)
{
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3|GPIO4);

  led_off(LED_GREEN);
  led_off(LED_RED);
}

/** Turn off an LED.
 * \param led LED to turn off.
 */
void led_off(u8 led)
{
  switch (led) {
    case LED_1:
      gpio_set(GPIOC, GPIO3);
      break;
    case LED_2:
      gpio_set(GPIOC, GPIO4);
      break;
    default:
      break;
  }
}

/** Turn on an LED.
 * \param led LED to turn on.
 */
void led_on(u8 led)
{
  switch (led) {
    case LED_1:
      gpio_clear(GPIOC, GPIO3);
      break;
    case LED_2:
      gpio_clear(GPIOC, GPIO4);
      break;
    default:
      break;
  }
}

/** Toggle an LED.
 * If currently on, switches off, if off, switches on.
 * \param led LED to toggle.
 */
void led_toggle(u8 led)
{
  switch (led) {
    case LED_1:
      gpio_toggle(GPIOC, GPIO3);
      break;
    case LED_2:
      gpio_toggle(GPIOC, GPIO4);
      break;
    default:
      break;
  }
}

/** \} */

/** \} */
