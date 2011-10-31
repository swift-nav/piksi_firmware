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

#include "../swift_nap.h"
#include "exti.h"
#include "leds.h"

u32 CIE[16], CQE[16], CIP[16], CQP[16], CIL[16], CQL[16];
u8 exti_count = 0;

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


/*void exti0_isr()*/
/*{*/
  /*uint32_t code_phase[2], carrier_phase[2];*/
  
  /*carrier_phase[1] = swift_nap_read(0, 0); // 8 MSB*/
  /*carrier_phase[1] = swift_nap_read(0, 1); // 32 LSB*/
  /*code_phase[1] = swift_nap_read(0, 10);   // 10 MSB*/
  /*code_phase[0] = swift_nap_read(0, 11);   // 32 LSB*/

  /*[> Do tracking loops <]*/

  /*swift_nap_write(*/
/*}*/

s32 sign_extend(u32 n, u8 bits)
{
  if (n & (1 << (bits-1)))
    return ((s32)(n << (32-bits)) >> (32-bits));
  
  return (s32)n;
}

void exti0_isr()
{

  if (exti_count < 16) {
    /*CIE[exti_count] = swift_nap_read(0, 17);*/
    /*CQE[exti_count] = swift_nap_read(0, 18);*/
    /*CIP[exti_count] = swift_nap_read(0, 19);*/
    /*CQP[exti_count] = swift_nap_read(0, 20);*/
    /*CIL[exti_count] = swift_nap_read(0, 21);*/
    /*CQL[exti_count] = swift_nap_read(0, 22);*/
    CIP[exti_count] = sign_extend(swift_nap_read(0, 19), 22);
    /*CQP[exti_count] = sign_extend(swift_nap_read(0, 20), 22);*/
    /*CIE[exti_count] = sign_extend(swift_nap_read(0, 17), 22);*/
    /*CQE[exti_count] = sign_extend(swift_nap_read(0, 18), 22);*/
    /*CIL[exti_count] = sign_extend(swift_nap_read(0, 21), 22);*/
    /*CQL[exti_count] = sign_extend(swift_nap_read(0, 22), 22);*/
    exti_count++;
  }

  led_toggle(LED_GREEN);
  exti_reset_request(EXTI0);
}
