/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdio.h>
#include "3drradio.h"

#define WAIT_FOR_3DR_MS 1100
#define WAIT_BETWEEN_COMMANDS 50
#define RADIO_RETRY_COUNT 1
#define FINAL_BAUDRATE 115200

/* see SiK firmware, serial.c, serial_rates[] */
static const u32 baud_rates[] = {57600, 115200};//, 230400, 38400, 19200};

static char const * commands[] = {
  "AT&F\r\n",          /* Reset to factory defaults */
  "ATS1=115\r\n",      /* Set to 115200 baud */
  "ATS2=128\r\n",      /* Set to 128kbps air rate */
  "ATS5=1\r\n",        /* Turn on ECC */
  "AT&W\r\n",          /* Write to EEPROM */
  "ATZ\r\n",           /* Reboot radio! */
};

bool busy_wait_for_ok(u32 usart, u32 ms)
{
  bool found_ok = false;
  u16 recv = 0;
  u8 step = 0;

  systime_t start_ticks = chTimeNow();
  while((step < 2) && (chTimeNow() - start_ticks < MS2ST(ms))) {
    recv = usart_recv(usart);
    switch (recv) {
      case 0:
        step = 0;
        break;
      case 'O':
        step = 1;
        break;
      case 'K':
        step = 2;
        found_ok = true;
        break;
    }
  }

  return found_ok;
}

/**

SMART THINGS TO DO:

- READ THE FIRMWARE
- RESET TO FACTORY DEFAULTS IF WE WANT
- GET RSSI REPORTS

*
*/
u32 radio_preconfigure_hook(u32 usart)
{
  bool found_radio = false;
  u8 tries = 0;
  u8 baud_index = 0;
  u32 baud_rate = 0;
  while (!found_radio && baud_index < (sizeof(baud_rates)/sizeof(baud_rates[0]))) {
    tries = 0;

    baud_rate = baud_rates[baud_index];
    baud_index++;

    /* Configure the UART for the current baudrate */
    usart_disable(usart);
    usart_set_parameters(usart, baud_rate);

    /* Try to get radio into AT command mode */
    while (!found_radio && (tries < RADIO_RETRY_COUNT)) {
      tries++;

      usart_send_blocking(usart, '+');
      usart_send_blocking(usart, '+');
      usart_send_blocking(usart, '+');

      /* Try to receive an "OK" back */
      found_radio = busy_wait_for_ok(usart, WAIT_FOR_3DR_MS);
    }

  }

  if (found_radio) {

    u8 command_index = 0;
    while (command_index < (sizeof(commands)/sizeof(char*))) {
      char* command = commands[command_index];

      while (*command != 0) {
        u16 c = (uint8_t)*command;
        usart_send_blocking(usart, c);
        command++;
      }

      /* Busy-wait for an OK. Doesn't matter if we don't always get it. */
      busy_wait_for_ok(usart, WAIT_BETWEEN_COMMANDS);
      command_index++;
    }

    /* Configure the UART for the current baudrate */
    usart_disable(usart);
    usart_set_parameters(usart, FINAL_BAUDRATE);


    return baud_rate;

  }
  return 0;

}
