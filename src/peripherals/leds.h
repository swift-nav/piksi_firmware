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

#ifndef SWIFTNAV_LEDS_H
#define SWIFTNAV_LEDS_H

#include <libswiftnav/common.h>

/** \addtogroup leds
 * \{ */

#define LED_GREEN LED_1
#define LED_RED   LED_2
#define LED_1     1
#define LED_2     2

/** \} */

void led_setup(void);
void led_on(u8 led);
void led_off(u8 led);
void led_toggle(u8 led);

#endif

