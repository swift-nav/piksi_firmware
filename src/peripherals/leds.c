/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

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
    palSetLine(LINE_LED1);
    break;

  case LED_2:
    palSetLine(LINE_LED2);
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
    palClearLine(LINE_LED1);
    break;

  case LED_2:
    palClearLine(LINE_LED2);
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
    palToggleLine(LINE_LED1);
    break;

  case LED_2:
    palToggleLine(LINE_LED2);
    break;

  default:
    break;
  }
}

/** \} */

/** \} */

