/*
 * bootloader for the Swift Navigation Piksi GPS Receiver
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2013-2014 Swift Navigation Inc <www.swift-nav.com>
 *
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * Based on luftboot, a bootloader for the Paparazzi UAV project.
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <libopencm3/cm3/scb.h>

#include "main.h"
#include "sbp.h"
#include "board/leds.h"
#include "peripherals/spi.h"
#include "board/nap/nap_common.h"
#include "flash_callbacks.h"

#define APP_ADDRESS   0x08004000
#define STACK_ADDRESS 0x10010000

u8 bootloader_version = 0;
u8 host_wants_bootload = 0;
u8 current_app_valid = 0;

/* Swap printf calls for this function at link time to save memory in the
 * bootloader (-Wl,-wrap,printf in linker flags).
 */
int __wrap_printf(const char *format __attribute__((unused)), ...)
{
  return 0;
}

void jump_to_app_callback(u8 buff[] __attribute__((unused)))
{
  /* Disable peripherals used in the bootloader. */
  sbp_disable();
  spi_deactivate();
  nap_conf_b_set();
  /* Set vector table base address. */
  SCB_VTOR = APP_ADDRESS & 0x1FFFFF00;
  /* Initialise master stack pointer. */
  asm volatile ("msr msp, %0"::"g"(*(volatile u32*)APP_ADDRESS));
  /* Jump to application. */
  (*(void(**)())(APP_ADDRESS + 4))();
}

void receive_handshake_callback(u8 buff[] __attribute__((unused)))
{
  /* Disable FPGA configuration and set up SPI in case we want to flash M25. */
  spi_setup();
  flash_callbacks_register();
  host_wants_bootload = 1;
}

u32 send_handshake(void)
{
  return sbp_send_msg(MSG_BOOTLOADER_HANDSHAKE,1,&bootloader_version);
}

int main(void)
{
  /* Disable FPGA configuration. */
  nap_conf_b_setup();
  nap_conf_b_clear();

  /* Setup and turn on LEDs. */
  led_setup();
  led_off(LED_GREEN);
  led_off(LED_RED);

  /* Setup UART and SBP interface for transmitting and receiving callbacks. */
  sbp_setup(0);

  /* Add callback for jumping to application after bootloading is finished. */
  static msg_callbacks_node_t jump_to_app_node;
  sbp_register_callback(MSG_BOOTLOADER_JUMP_TO_APP, &jump_to_app_callback,
                        &jump_to_app_node);

  /* Add callback for host to tell bootloader it wants to load program. */
  static msg_callbacks_node_t receive_handshake_node;
  sbp_register_callback(MSG_BOOTLOADER_HANDSHAKE,&receive_handshake_callback,
                        &receive_handshake_node);

  /* Is current application we have in flash valid? Check this by seeing if
   * the first address of the application contains the correct stack address.
   */
  current_app_valid = (*(volatile u32*)APP_ADDRESS == STACK_ADDRESS) ? 1:0;

  /*
   * Wait a bit for response from host. If it doesn't respond by calling
   * receive_handshake_callback and we have a valid application, then boot
   * the application.
   */
	for (u64 i=0; i<400000; i++){
    DO_EVERY(3000,
      led_toggle(LED_RED);
      send_handshake();
    );
    sbp_process_messages(); /* To service receive_handshake_callback. */
    if (host_wants_bootload) break;
  }
  led_off(LED_GREEN);
  led_off(LED_RED);
  if ((host_wants_bootload) || !(current_app_valid)){
    /*
     * We expect host application passing firmware data to call
     * jump_to_app_callback to break us out of this while loop after it has
     * finished sending flash programming callbacks
     */
    while(1){
      sbp_process_messages();
      DO_EVERY(3000,
        led_toggle(LED_GREEN);
        led_toggle(LED_RED);
        /*
         * In case host application was started after we entered the loop. It
         * expects to get a bootloader handshake message before it will send
         * flash programming callbacks.
         */
        DO_EVERY(10,
          send_handshake();
        );
      );
    }
  }

  /* Host didn't want to update - boot the existing application. */
  jump_to_app_callback(NULL);

  return 0;
}
