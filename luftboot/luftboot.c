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

#include <stdio.h>
#include <libopencm3/stm32/f4/scb.h>

#include "main.h"
#include "hw/leds.h"
#include "debug.h"
#include "hw/stm_flash.h"

#define APP_ADDRESS	0x08010000

void jump_to_app_callback(u8 buff[]){
  /* Doing something with buff so as not to get compile error. 
   * Cludge, will change later */
  buff[0] = buff[0] & 0xFF;
  /* Set vector table base address */
  SCB_VTOR = APP_ADDRESS & 0xFFFF;
  /* Initialise master stack pointer */
  asm volatile ("msr msp, %0"::"g"
      (*(volatile u32*)APP_ADDRESS));
  /* Jump to application */
  (*(void(**)())(APP_ADDRESS + 4))();
}

int main(void)
{
  /* Setup and turn on LEDs */
  led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Setup UART and debug interface for 
   * transmitting and receiving flash callbacks */
  debug_setup();

  /* Add callbacks for erasing, programming and reading flash */
  stm_flash_callbacks_setup();

  /* Add callback for jumping to application after bootloading is finished */
  static msg_callbacks_node_t jump_to_app_node;
  debug_register_callback(MSG_JUMP_TO_APP, &jump_to_app_callback, &jump_to_app_node);
	
  /* Process debug messages */
	while (1){
    debug_process_messages();
    DO_EVERY(100000,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
  }

  ///* Boot the application if it's valid */
  //if ((*(volatile u32*)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
  //  /* Set vector table base address */
  //  SCB_VTOR = APP_ADDRESS & 0xFFFF;
  //  /* Initialise master stack pointer */
  //  asm volatile ("msr msp, %0"::"g"
  //      (*(volatile u32*)APP_ADDRESS));
  //  /* Jump to application */
  //  (*(void(**)())(APP_ADDRESS + 4))();
  //}
}
