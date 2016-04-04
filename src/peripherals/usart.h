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

#include <hal.h>
#include <ch.h>

/** \addtogroup io
 * \{ */

 /** Message and baud rate settings for a USART. */
typedef struct {
  enum {
    SBP,
    NMEA,
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

#define USART_DEFAULT_BAUD_FTDI 1000000
#define USART_DEFAULT_BAUD_TTL  115200
#define USART_DEFAULT_BAUD_RADIO 57600

/** USART DMA state structure. */
typedef struct {
  void *sd;              /**< Pointer to board specific driver structure */
  bool configured;
  /** USART RX stats structure. */
  struct usart_stats {
    u32 byte_counter;    /**< Counts the number of bytes received since
                              statistics were last calculated */
    u32 errors;          /**< Counts the number of USART DMA errors */
    u32 last_byte_ticks; /**< Tick count of the last time throughput statistics
                              were calculated */
  } rx;
  struct usart_stats tx;
  binary_semaphore_t claimed; /**< Taken by module when channel is in use. */
  const void *claimed_by;
  int claim_nest;
} usart_state;

/** \} */

/** \} */

extern const u8 dma_irq_lookup[2][8];

extern usart_state ftdi_state;
extern usart_state uarta_state;
extern usart_state uartb_state;

void usarts_setup();

void usarts_enable(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud, bool do_preconfigure_hooks);
void usarts_disable(void);

bool usart_claim(usart_state* s, const void *module);
void usart_release(usart_state* s);

u32 usart_tx_n_free(usart_state* s);
u32 usart_write(usart_state* s, const u8 data[], u32 len);

u32 usart_n_read(usart_state* s);
u32 usart_read(usart_state* s, u8 data[], u32 len);
u32 usart_read_timeout(usart_state* s, u8 data[], u32 len, u32 timeout);

float usart_throughput(struct usart_stats* s);

/* Support functions to be provided by the board specific implementation */
void usart_support_init(void);
void usart_support_set_parameters(void *sd, u32 baud);
void usart_support_disable(void *sd);
u32 usart_support_n_read(void *sd);
u32 usart_support_tx_n_free(void *sd);
u32 usart_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout);
u32 usart_support_write(void *sd, const u8 data[], u32 len);

#endif  /* SWIFTNAV_USART_H */

