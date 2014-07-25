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

#ifndef SWIFTNAV_USART_CHAT_H
#define SWIFTNAV_USART_CHAT_H

#include "peripherals/usart.h"

enum uart {
  UART_NONE = 0,
  UART_FTDI = 1,
  UARTA = 2,
  UARTB = 3,
};
#define TYPE_UART uart_setting_type()

int uart_setting_type(void);
void usart_escape(enum uart u);
bool usart_sendwait(enum uart u, const char *send, const char *wait, u32 timeout);

static inline usart_tx_dma_state * uart_tx_state(enum uart u)
{
  usart_tx_dma_state * const state[] = {
    NULL, &ftdi_tx_state, &uarta_tx_state, &uartb_tx_state,
  };
  return state[u];
}

static inline usart_rx_dma_state * uart_rx_state(enum uart u)
{
  usart_rx_dma_state * const state[] = {
    NULL, &ftdi_rx_state, &uarta_rx_state, &uartb_rx_state,
  };
  return state[u];
}

#endif

