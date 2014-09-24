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

#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "error.h"
#include "board/leds.h"
#include "peripherals/usart.h"

u8 guard_below[30];
u8 buff_out[256];
u8 guard_above[30];

void timer_setup() {
  RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
  timer_set_prescaler(TIM2, 1);
  timer_set_period(TIM2, rcc_ppre1_frequency / 100);
  timer_enable_irq(TIM2, TIM_DIER_UIE);

  TIM2_CNT = 0;
  timer_enable_counter(TIM2);
  nvic_enable_irq(NVIC_TIM2_IRQ);
}

void tim2_isr() {
  timer_clear_flag(TIM2, TIM_SR_UIF);
  led_toggle(LED_GREEN);

  /* Random transmit length. */
  u32 len = (u32)rand() % 256;
  if(sbp_send_msg(0x22, len, buff_out))
    screaming_death("sbp_send_msg failed in tim2_isr");
}

int main(void)
{
  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- SWIFT BINARY PROTOCOL TX STRESS TEST ---\n");

  u32 len;

  for (u8 i=0; i<30; i++) {
    guard_below[i] = 0;
    guard_above[i] = 0;
  }

  for (u32 i=0; i<256; i++)
    buff_out[i] = (u8)i;

  while(1) {
    /* Random transmit length. */
    len = (u32)rand() % 256;
    while(sbp_send_msg(0x22, len, buff_out));

    /* Check the guards for buffer over/underrun. */
    for (u8 i=0; i<30; i++) {
      if (guard_below[i] != 0)
        screaming_death("Detected buffer underrun in guard area\n");
      if (guard_above[i] != 0)
        screaming_death("Detected buffer overrun in guard area\n");
    }

    /* Introduce some timing jitter. */
    u32 jitter_delay = ((u32)rand() % 20000);
    for (u32 i = 0; i < jitter_delay; i++)
    /*for (u32 i = 0; i < 1000; i++)*/
      __asm__("nop");
  }
while (1);

	return 0;
}

