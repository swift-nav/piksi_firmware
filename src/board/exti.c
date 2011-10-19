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

#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/nvic.h>

#include "exti.h"

void exti_setup()
{
  // Signal from the FPGA is on PA0.
  
  RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable clock to GPIOA
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable clock to SYSCFG "peripheral", which we think contains the EXTI functionality.

	/* Enable EXTI0 interrupt */
	nvic_enable_irq(NVIC_EXTI0_IRQ);

  exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
}

/*
void exti0_isr()
{

  if (EXTI_PR & EXTI0)
    led_on(LED_RED);
  else
    led_off(LED_RED);
  exti_reset_request(EXTI0);

  //led_toggle(LED_GREEN);

}
*/

