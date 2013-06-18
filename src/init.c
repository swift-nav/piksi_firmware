/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "sbp.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/nap_common.h"

const clock_scale_t hse_16_368MHz_in_65_472MHz_out_3v3 =
{ /* 65.472 MHz */
  .pllm = 16,
  .plln = 256,
  .pllp = 4,
  .pllq = 7,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_2WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 16368000,
};

const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3 =
{ /* 130.944 MHz (Overclocked!!) */
  .pllm = 16,
  .plln = 256,
  .pllp = 2,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_3WS,
  .apb1_frequency = 32736000,
  .apb2_frequency = 32736000,
};

const clock_scale_t hse_16_368MHz_in_120_203MHz_out_3v3 =
{ /* 120.203 MHz (USB OK but APB freq */
  .pllm = 16,
  .plln = 235,
  .pllp = 2,
  .pllq = 5,
  .hpre = RCC_CFGR_HPRE_DIV_2,
  .ppre1 = RCC_CFGR_PPRE_DIV_2,
  .ppre2 = RCC_CFGR_PPRE_DIV_2,
  .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_3WS,
  .apb1_frequency = 30050625,
  .apb2_frequency = 30050625,
};

void init()
{
  /* Delay as some programmers reset the STM twice. */
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);

  nap_setup();
  nap_reset();

  sbp_setup(1);

  m25_setup();
}

