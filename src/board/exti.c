/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/nvic.h>

#include "../swift_nap.h"
#include "exti.h"
#include "leds.h"

u32 exti_count = 0;

void exti_setup()
{
  // Signal from the FPGA is on PC6.
  
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;   // Enable clock to GPIOA
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable clock to SYSCFG "peripheral", which we think contains the EXTI functionality.

  exti_select_source(EXTI6, GPIOC);
	exti_set_trigger(EXTI6, EXTI_TRIGGER_RISING);
  exti_reset_request(EXTI6);
	exti_enable_request(EXTI6);

	/* Enable EXTI6 interrupt */
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

}

void exti9_5_isr()
{
  exti_reset_request(EXTI6);
  led_on(LED_GREEN);
  /*gpio_set(GPIOC, GPIO11|GPIO10);*/
  /*gpio_clear(GPIOC, GPIO11|GPIO10);*/
  /*exti_count = timing_count();*/
  exti_count++;
}

u32 last_exti_count() {
  return exti_count;
}

void wait_for_exti() {
  u32 last_last_exti = last_exti_count();
  while(last_exti_count() == last_last_exti);
}

