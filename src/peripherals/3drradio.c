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

#include <string.h>

#include <ch.h>
#include <hal.h>

#include <libswiftnav/logging.h>

#include "3drradio.h"
#include "../settings.h"

#define WAIT_FOR_3DR_MS 1200
#define WAIT_BETWEEN_COMMANDS 500
#define WAIT_BETWEEN_BYTES 100
#define RADIO_RETRY_COUNT 1
#define FINAL_BAUDRATE 115200
#define MAXLEN 256

/* see SiK firmware, serial.c, serial_rates[] */
static const u32 baud_rates[] = {115200, 57600};//, 230400, 38400, 19200};

/* This is the command string we send to the radios */
static char commandstr[MAXLEN] = "AT&F,ATS1=57,ATS2=64,ATS5=0,AT&W,ATZ";

/**
* This function hooks into the UART setup code before DMA gets enabled,
* and configures any 3DR radio it finds on the given uart.
*/
void radio_preconfigure_hook(enum uart u, u32 default_baud, char* uart_name)
{

  /** TODO:
  * Future features we might consider
  * - READ THE FIRMWARE
  * - RESET TO FACTORY DEFAULTS IF WE WANT
  * - GET RSSI REPORTS
  */

  bool found_radio = false;
  u8 tries = 0;
  u8 baud_index = 0;
  u32 baud_rate = 0;

  /* First we attempt to find a radio at all possible baudrates */

  while (!found_radio && baud_index < (sizeof(baud_rates)/sizeof(baud_rates[0]))) {
    tries = 0;
    baud_rate = baud_rates[baud_index];
    baud_index++;

    /* Configure the UART for the current baudrate */
    usart_support_set_parameters(uart_state(u)->sd, baud_rate);

    /* Try to get a radio into AT command mode */
    while (!found_radio && (tries < RADIO_RETRY_COUNT)) {
      tries++;
      found_radio = usart_sendwait(u, "+++", "OK\r\n", WAIT_FOR_3DR_MS);
    }

  }

  /* If we found a radio, we send it a configuration string. */
  if (found_radio) {
    log_info("Telemetry radio found on %s at baudrate %lu, "
             "sending configuration string.\n", uart_name, baud_rate);

    char* command = commandstr;
    while (*command != 0) {

      if (*command == ',') {
        usart_sendwait(u, "\r\n", "OK\r\n", WAIT_BETWEEN_COMMANDS);
      } else {
        usart_write(uart_state(u), (u8*)command, 1);
      }

      command++;
    }

    usart_sendwait(u, "\r\n", "OK\r\n", WAIT_BETWEEN_COMMANDS);

  } else {
    log_info("No telemetry radio found on %s, skipping configuration.",
             uart_name);
  }

  /* Reset the UART to the original baudrate. */
  usart_support_set_parameters(uart_state(u)->sd, default_baud);

}

void radio_setup()
{
  SETTING("telemetry_radio", "configuration_string", commandstr, TYPE_STRING);
}

