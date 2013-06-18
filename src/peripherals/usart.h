/*
 * Copyright (C) 2012-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_USART_H
#define SWIFTNAV_USART_H

#include <libopencm3/cm3/common.h>

/** \addtogroup peripherals
 * \{ */

/** \addtogroup usart
 * \{ */

#define USART_TX_BUFFER_LEN 4096
#define USART_RX_BUFFER_LEN 4096

#define USART_DEFAULT_BAUD 1000000

/** USART RX DMA state structure. */
typedef struct {
  u8 buff[USART_RX_BUFFER_LEN]; /**< USART RX DMA buffer. DMA xfers from USART to buffer, message processing routine reads out of buffer. */
  u32 rd;       /**< Address of next byte to read out of buffer. */
  /* TODO : is u32 big enough for rd_wraps and wr_wraps? */
  u32 rd_wraps; /**< Number of times rd has wrapped around the buffer. */
  u32 wr_wraps; /**< Number of times wr has wrapped around the buffer. */

  u32 dma;      /**< DMA for particular USART. */
  u32 usart;    /**< USART peripheral this state serves. */
  u8 stream;    /**< DMA stream for this USART. */
  u8 channel;   /**< DMA channel for this USART. */
} usart_rx_dma_state;

/** USART TX DMA state structure. */
typedef struct {
  u8 buff[USART_TX_BUFFER_LEN]; /**< USART TX DMA buffer. DMA xfers from buffer to USART_DR. */
  u32 rd;       /**< Address of next byte to read out of buffer. */
  u32 wr;       /**< Next buffer address to write to. */
  u32 xfer_len; /**< Number of bytes to DMA from buffer to USART_DR. */

  u32 dma;      /**< DMA for particular USART. */
  u32 usart;    /**< USART peripheral this state serves. */
  u8 stream;    /**< DMA stream for this USART. */
  u8 channel;   /**< DMA channel for this USART. */
} usart_tx_dma_state;

/** \} */

/** \} */

extern const u8 dma_irq_lookup[2][8];

extern usart_tx_dma_state ftdi_tx_state;
extern usart_rx_dma_state ftdi_rx_state;
extern usart_tx_dma_state uarta_tx_state;
extern usart_rx_dma_state uarta_rx_state;
extern usart_tx_dma_state uartb_tx_state;
extern usart_rx_dma_state uartb_rx_state;

void usarts_setup(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud);
void usarts_disable();

void usart_tx_dma_setup(usart_tx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
void usart_tx_dma_disable(usart_tx_dma_state* s);
u32 usart_tx_n_free(usart_tx_dma_state* s);
void usart_tx_dma_isr(usart_tx_dma_state* s);
u32 usart_write_dma(usart_tx_dma_state* s, u8 data[], u32 len);

void usart_rx_dma_setup(usart_rx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
void usart_rx_dma_disable(usart_rx_dma_state* s);
void usart_rx_dma_isr(usart_rx_dma_state* s);
u32 usart_n_read_dma(usart_rx_dma_state* s);
u32 usart_read_dma(usart_rx_dma_state* s, u8 data[], u32 len);

#endif /* SWIFTNAV_USART_H */

