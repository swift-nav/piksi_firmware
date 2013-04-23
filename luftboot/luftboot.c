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
#define STACK_ADDRESS 0x10010000

u8 pc_wants_bootload = 0;

void jump_to_app_callback(u8 buff[] __attribute__((unused))){
  /* Should we disable interrupts here? */
  /* Set vector table base address */
  SCB_VTOR = APP_ADDRESS & 0xFFFF;
  /* Initialise master stack pointer */
  asm volatile ("msr msp, %0"::"g"(*(volatile u32*)APP_ADDRESS));
  /* Jump to application */
  (*(void(**)())(APP_ADDRESS + 4))();
}

void pc_wants_bootload_callback(u8 buff[] __attribute__((unused))){
  pc_wants_bootload = 1;
}

int main(void)
{
  /* Setup and turn on LEDs */
  led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Setup UART and debug interface for 
   * transmitting and receiving callbacks */
  debug_setup();

  /* Add callbacks for erasing, programming and reading flash */
  stm_flash_callbacks_setup();

  /* Add callback for jumping to application after bootloading is finished */
  static msg_callbacks_node_t jump_to_app_node;
  debug_register_callback(MSG_JUMP_TO_APP, &jump_to_app_callback, 
    &jump_to_app_node);

  /* Add callback for PC to tell bootloader it wants to load program */
  static msg_callbacks_node_t pc_wants_bootload_node;
  debug_register_callback(MSG_BOOTLOADER_HANDSHAKE,&pc_wants_bootload_callback,
    &pc_wants_bootload_node);

  /* Send message to PC indicating bootloader is ready to load program */
  debug_send_msg(MSG_BOOTLOADER_HANDSHAKE,0,0);

  /* Is current application we are programmed with valid? */
  u8 current_app_valid = (*(volatile u32*)APP_ADDRESS == STACK_ADDRESS) ? 1:0;

  /* 
   * Wait a bit for response from PC. If it doesn't respond by calling
   * pc_wants_bootload_callback and we have a valid application, 
   * then boot the application 
   * TODO : might as well make this as long as FPGA takes to configure itself
   *        from the configuration flash, as it doesn't add to the startup time
   */
	for (u64 i=0; i<100000; i++){
    DO_EVERY(3000,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
      debug_send_msg(MSG_BOOTLOADER_HANDSHAKE,0,0);
    );
    debug_process_messages(); // to service pc_wants_bootload_callback
    if (pc_wants_bootload)
      break;
  }
  if ((pc_wants_bootload) || !(current_app_valid)){
//  if (pc_wants_bootload){
    /*
     * We expect PC application passing application data to call 
     * jump_to_app_callback to break us out of this while loop after it has
     * finished sending flash programming callbacks
     */
    while(1){
      debug_process_messages();
      DO_EVERY(50000,
        led_toggle(LED_GREEN);
        led_toggle(LED_RED);
        /* 
         * In case PC application was started after we entered the loop. It
         * is expecting to get a bootloader handshake message before it will
         * send flash programming callbacks 
         */
        debug_send_msg(MSG_BOOTLOADER_HANDSHAKE,0,0);
      );
    }
  }

  /* Looks like the PC didn't want to update - boot the existing application */
  /* Should we disable interrupts here? */
  /* Set vector table base address */
  SCB_VTOR = APP_ADDRESS & 0xFFFF;
  /* Initialise master stack pointer */
  asm volatile ("msr msp, %0"::"g"(*(volatile u32*)APP_ADDRESS));
  /* Jump to application */
  (*(void(**)())(APP_ADDRESS + 4))();

  while(1);
}
