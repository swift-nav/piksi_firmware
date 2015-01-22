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

#ifndef SWIFTNAV_USART_H
#define SWIFTNAV_USART_H

#include <libswiftnav/common.h>
#include "settings.h"

#include "ch.h"

#define dma2_stream6_isr Vector154
#define dma2_stream1_isr Vector124
#define dma2_stream7_isr Vector158
#define dma2_stream2_isr Vector128
#define dma1_stream3_isr Vector78
#define dma1_stream1_isr Vector70

/** \addtogroup io
 * \{ */

 /** Message and baud rate settings for a USART. */
typedef struct {
  enum {
    SBP,
    NMEA,
    RTCM,
  } mode; /**< Communication mode : Swift Binary Protocol or NMEA */
  u32 baud_rate;
  u32 sbp_message_mask;
  u8  configure_telemetry_radio_on_boot;
} usart_settings_t;

/** Message and baud rate settings for all USARTs. */
extern usart_settings_t ftdi_usart;
extern usart_settings_t uarta_usart;
extern usart_settings_t uartb_usart;

/** \} */

/** \addtogroup peripherals
 * \{ */

/** \addtogroup usart
 * \{ */

#define USART_DMA_ISR_PRIORITY 7

#define USART_TX_BUFFER_LEN 2048
#define USART_RX_BUFFER_LEN 2048

#define USART_DEFAULT_BAUD_FTDI 1000000
#define USART_DEFAULT_BAUD_TTL  115200

/** USART DMA state structure. */
typedef struct {
  bool configured;
  /** USART RX DMA state structure. */
  struct usart_rx_dma_state {
    /** USART RX DMA buffer. DMA xfers from USART to buffer, message processing
     * routine reads out of buffer. */
    u8 buff[USART_RX_BUFFER_LEN];
    u32 rd;       /**< Address of next byte to read out of buffer.  */
    /* TODO : is u32 big enough for rd_wraps and wr_wraps? */
    u32 rd_wraps; /**< Number of times rd has wrapped around the buffer. */
    u32 wr_wraps; /**< Number of times wr has wrapped around the buffer. */

    u32 dma;      /**< DMA for particular USART. */
    u32 usart;    /**< USART peripheral this state serves. */
    u8 stream;    /**< DMA stream for this USART. */
    u8 channel;   /**< DMA channel for this USART. */

    u32 byte_counter;    /**< Counts the number of bytes received since
                              statistics were last calculated */
    u32 errors;          /**< Counts the number of USART DMA errors */
    u32 last_byte_ticks; /**< Tick count of the last time throughput statistics
                              were calculated */

    BinarySemaphore ready_sem; /**< Semaphore released when ready to read. */
  } rx;
  /** USART TX DMA state structure. */
  struct usart_tx_dma_state {
    /** USART TX DMA buffer. DMA xfers from buffer to USART_DR. */
    u8 buff[USART_TX_BUFFER_LEN];
    u32 rd;       /**< Address of next byte to read out of buffer. */
    u32 wr;       /**< Next buffer address to write to. */
    u32 xfer_len; /**< Number of bytes to DMA from buffer to USART_DR. */

    u32 dma;      /**< DMA for particular USART. */
    u32 usart;    /**< USART peripheral this state serves. */
    u8 stream;    /**< DMA stream for this USART. */
    u8 channel;   /**< DMA channel for this USART. */

    u32 byte_counter;    /**< Counts the number of bytes received since
                              statistics were last calculated */
    u32 errors;          /**< Counts the number of USART DMA errors */
    u32 last_byte_ticks; /**< Tick count of the last time throughput statistics
                              were calculated */
  } tx;
  BinarySemaphore claimed; /**< Taken by module when channel is in use. */
  const void *claimed_by;
  int claim_nest;
} usart_dma_state;

typedef struct usart_tx_dma_state usart_tx_dma_state;
typedef struct usart_rx_dma_state usart_rx_dma_state;

/** \} */

/** \} */

extern const u8 dma_irq_lookup[2][8];

extern usart_dma_state ftdi_state;
extern usart_dma_state uarta_state;
extern usart_dma_state uartb_state;

void usarts_setup();
bool baudrate_change_notify(struct setting *s, const char *val);

void usarts_enable(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud, bool do_preconfigure_hooks);
void usarts_disable(void);

bool usart_claim(usart_dma_state* s, const void *module);
void usart_release(usart_dma_state* s);

void usart_set_parameters(u32 usart, u32 baud);

void usart_tx_dma_setup(usart_tx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
void usart_tx_dma_disable(usart_tx_dma_state* s);
u32 usart_tx_n_free(usart_tx_dma_state* s);
void usart_tx_dma_isr(usart_tx_dma_state* s);
u32 usart_write_dma(usart_tx_dma_state* s, const u8 data[], u32 len);
float usart_tx_throughput(usart_tx_dma_state* s);

void usart_rx_dma_setup(usart_rx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
void usart_rx_dma_disable(usart_rx_dma_state* s);
void usart_rx_dma_isr(usart_rx_dma_state* s);
u32 usart_n_read_dma(usart_rx_dma_state* s);
u32 usart_read_dma(usart_rx_dma_state* s, u8 data[], u32 len);
u32 usart_read_dma_timeout(usart_rx_dma_state* s, u8 data[], u32 len, u32 timeout);
float usart_rx_throughput(usart_rx_dma_state* s);

#endif  /* SWIFTNAV_USART_H */

