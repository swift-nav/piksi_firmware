/*
 * Copyright (C) 2014 Swift Navigation Inc.
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
#include <stdio.h>

#include "peripherals/usart.h"
#include "peripherals/usart_chat.h"
#include "settings.h"

#define TIMEOUT_CHAR 100

int uart_setting_type(void)
{
  static const char const * uart_enum[] =
    {"NONE", "FTDI", "UARTA", "UARTB", NULL};
  static struct setting_type uart;
  static int type;

  if (type == 0)
    type = settings_type_register_enum(uart_enum, &uart);
  return type;
}

void usart_escape(enum uart u)
{
  if (u == UART_NONE)
    return;
  chThdSleepMilliseconds(1100);
  usart_write_dma(uart_tx_state(u), (const u8*)"+", 1);
  chThdSleepMilliseconds(100);
  usart_write_dma(uart_tx_state(u), (const u8*)"+", 1);
  chThdSleepMilliseconds(100);
  usart_write_dma(uart_tx_state(u), (const u8*)"+", 1);
  chThdSleepMilliseconds(1100);
}

bool usart_sendwait(enum uart u, const char *send, const char *wait, u32 timeout)
{
  int i;
  u8 c;

  if (u == UART_NONE)
    return false;

  /* Flush out anything in the receive buffer */
  if (send && send[0]) {
    while (usart_n_read_dma(uart_rx_state(u))) {
      usart_read_dma(uart_rx_state(u), &c, 1);
    }

    usart_write_dma(uart_tx_state(u), (const u8*)send, strlen(send));

    /* Wait for command echo */
    for (i = 0; send[i]; i++) {
      usart_read_dma_timeout(uart_rx_state(u), &c, 1, TIMEOUT_CHAR);
      if (c != send[i]) {
        printf("No echo: '%s'\n", send);
        return false;
      }
    }
  }

  /* Round timeout up to multiple of TIMEOUT_CHAR */
  timeout += TIMEOUT_CHAR - 1;
  timeout -= timeout % TIMEOUT_CHAR;
  i = 0;
  do {
    if (usart_read_dma_timeout(uart_rx_state(u), &c, 1, TIMEOUT_CHAR) == 0)
      timeout -= TIMEOUT_CHAR;
    if (c == wait[i]) {
      i++;
    } else {
      i = 0;
    }
  } while (wait[i] && timeout);

  return wait[i] == 0;
}

