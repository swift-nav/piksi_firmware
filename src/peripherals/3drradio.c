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

#include <libopencm3/stm32/f4/usart.h>

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

/** Wait until the UART has data ready to be received.
* or until ms time passes.
*
* THIS FUNCTION BLOCKS!
*
* \param usart   The libopencm3-defined UART base address to wait for
* \param ms      The maximum time in milliseconds to wait for data
*
* \return        True if data is available, False if timeout occurs
*/
bool usart_wait_recv_ready_with_timeout(uint32_t usart, u32 ms)
{
  /* Wait until the data has been transferred from the shift register. */

  systime_t start_ticks = chTimeNow();

  volatile uint32_t flag = 0;
  while (flag == 0 && (chTimeNow() - start_ticks < MS2ST(ms))) {
    flag = (USART_SR(usart) & USART_SR_RXNE);
  }
  return flag != 0;
}

/** Blocks until a given string appears on the uart,
* or until ms time passes.
*
* THIS FUNCTION BLOCKS!
*
* \param usart   The libopencm3-defined UART base address to wait for
* \param str     The null-terminated string to look for.
*                (Does not expect a null terminal from UART)
* \param ms      The maximum time to wait for the ENTIRE string to be received
* \return        True if an OK was found, false otherwise.
*/
bool busy_wait_for_str(u32 usart, char* str, u32 ms)
{
  u16 recv = 0;
  u8 step = 0;

  u8 len = strlen(str);
  systime_t start_ticks = chTimeNow();
  while ((step < len) && ((chTimeNow() - start_ticks) < MS2ST(ms))) {

    if (usart_wait_recv_ready_with_timeout(usart, WAIT_BETWEEN_BYTES)) {
      /* Cut out any parity bit if it exists. */
      recv = usart_recv(usart) & 0x00FF;
      if (recv == str[step]) {
        step++;
      } else {
        step = 0;
      }
    }
  }
  return step == len;
}

/** Blocks until the entire string is sent to the UART.
*/
void usart_send_str_blocking(u32 usart, char* str)
{
  while (*str != 0) {
    usart_send_blocking(usart, *str & 0x00FF);
    str++;
  }
  usart_wait_send_ready(usart);
}

/**
* This function hooks into the UART setup code before DMA gets enabled,
* and configures any 3DR radio it finds on the given uart.
*/
void radio_preconfigure_hook(u32 usart, u32 default_baud, char* uart_name)
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
    usart_set_parameters(usart, baud_rate);

    /* Try to get a radio into AT command mode */
    while (!found_radio && (tries < RADIO_RETRY_COUNT)) {
      tries++;
      usart_send_str_blocking(usart, "+++");
      found_radio = busy_wait_for_str(usart, "OK\r\n", WAIT_FOR_3DR_MS);
    }

  }

  /* If we found a radio, we send it a configuration string. */
  if (found_radio) {
    log_info("Telemetry radio found on %s at baudrate %lu, "
             "sending configuration string.\n", uart_name, baud_rate);

    char* command = commandstr;
    while (*command != 0) {

      if (*command == ',') {
        usart_send_str_blocking(usart, "\r\n");
        busy_wait_for_str(usart, "OK\r\n", WAIT_BETWEEN_COMMANDS);
      } else {
        u16 c = (uint8_t)*command;
        usart_send_blocking(usart, c);
      }

      command++;
    }

    usart_send_str_blocking(usart, "\r\n");
    busy_wait_for_str(usart, "\x00", WAIT_BETWEEN_COMMANDS);

  } else {
    log_info("No telemetry radio found on %s, skipping configuration.",
             uart_name);
  }

  /* Reset the UART to the original baudrate. */
  usart_set_parameters(usart, default_baud);

}

void radio_setup()
{
  SETTING("telemetry_radio", "configuration_string", commandstr, TYPE_STRING);
}

