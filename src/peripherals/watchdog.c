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

#include <hal.h>

#include "./watchdog.h"

#define COUNT_LENGTH 12
#define COUNT_MASK ((1 << COUNT_LENGTH)-1)

#define IWDG_KR_RESET			0xaaaa
#define IWDG_KR_UNLOCK			0x5555
#define IWDG_KR_START			0xcccc

#define IWDG_PR_DIV4			0x0
#define IWDG_PR_DIV8			0x1
#define IWDG_PR_DIV16			0x2
#define IWDG_PR_DIV32			0x3
#define IWDG_PR_DIV64			0x4
#define IWDG_PR_DIV128			0x5
#define IWDG_PR_DIV256			0x6

static WDGConfig wdg_config;

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
  uint32_t count, prescale, reload, exponent;

  /* Set the count to represent ticks of the 32kHz LSI clock */
  count = (period_ms << 5);

  /* Strip off the first 12 bits to get the prescale value required */
  prescale = (count >> 12);
  if (prescale > 256) {
  	exponent = IWDG_PR_DIV256; reload = COUNT_MASK;
  } else if (prescale > 128) {
  	exponent = IWDG_PR_DIV256; reload = (count >> 8);
  } else if (prescale > 64) {
  	exponent = IWDG_PR_DIV128; reload = (count >> 7);
  } else if (prescale > 32) {
  	exponent = IWDG_PR_DIV64;  reload = (count >> 6);
  } else if (prescale > 16) {
  	exponent = IWDG_PR_DIV32;  reload = (count >> 5);
  } else if (prescale > 8) {
  	exponent = IWDG_PR_DIV16;  reload = (count >> 4);
  } else if (prescale > 4) {
  	exponent = IWDG_PR_DIV8;   reload = (count >> 3);
  } else {
  	exponent = IWDG_PR_DIV4;   reload = (count >> 2);
  }

  /* Avoid the undefined situation of a zero count */
  if (count == 0) {
  	count = 1;
  }

  wdg_config.pr = exponent;
  wdg_config.rlr = reload & COUNT_MASK;
  wdgStart(&WDGD1, &wdg_config);
}

/** Clear (reset) the independent watchdog timer. */
void watchdog_clear(void)
{
  wdgReset(&WDGD1);
}

/** \} */
/** \} */
