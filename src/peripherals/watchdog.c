/* watchdog.c: Wrapper for watchdog timer peripheral
 *
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/iwdg.h>
#include "./watchdog.h"

/** \addtogroup peripherals
 * \{ */

/** \defgroup watchdog Watchdog
 * Functions to setup and use STM32F4 independent watchdog timer.
 * \{ */

/** Setup and enable the independent watchdog timer. 
 * \param period_ms Desired watchdog period in milliseconds.  Note that the LSI
 *   RC oscillator which drives the IWDG has a very loose timing spec.  Actual
 *   period may vary up to +88% -32% from this value.
 */
void watchdog_enable(uint32_t period_ms)
{
  iwdg_set_period_ms(period_ms);
  iwdg_start();
  iwdg_reset();
}

/** Clear (reset) the independent watchdog timer. */
void watchdog_clear(void)
{
  iwdg_reset();
}

/** \} */
/** \} */
