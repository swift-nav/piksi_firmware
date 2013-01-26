/*
 * This file is part of the Paparazzi UAV project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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

#include <string.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/scb.h>

#include "main.h"
#include "hw/leds.h"

#define APP_ADDRESS	0x08002000

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21 
#define CMD_ERASE	0x41 

//static struct {
//	u8 buf[sizeof(usbd_control_buffer)];
//	u16 len;
//	u32 addr;
//	u16 blocknum;
//} prog;

int main(void)
{
	/* Enable clock for the "force bootloader" pin bank and check for it */
//	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
//	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
//	gpio_clear(GPIOC, GPIO0);
//	if(gpio_get(GPIOC, GPIO0)) {
  /* Boot the application if it's valid */
  if ((*(volatile u32*)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
    /* Set vector table base address */
    SCB_VTOR = APP_ADDRESS & 0xFFFF;
    /* Initialise master stack pointer */
    asm volatile ("msr msp, %0"::"g"
        (*(volatile u32*)APP_ADDRESS));
    /* Jump to application */
    (*(void(**)())(APP_ADDRESS + 4))();
  }
//	}

//	rcc_clock_setup_in_hse_12mhz_out_72mhz();

//	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
//			GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
//	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
//			GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
//	gpio_toggle(GPIOC, GPIO2); /* LED2 on/off */
  
	
//	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8); 
//	systick_set_reload(900000);
//	systick_interrupt_enable();
//	systick_counter_enable();

  led_setup();
	
	while (1){
    DO_EVERY(100000,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
    __asm__("nop");
  }
}
