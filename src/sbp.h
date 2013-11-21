/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_H
#define SWIFTNAV_SBP_H

#include <libswiftnav/common.h>

#include "peripherals/usart.h"
#include "sbp_messages.h"

/** \addtogroup sbp
 * \{ */

#define SBP_HEADER_1  0xBE
#define SBP_HEADER_2  0xEF

#define SBP_MSG(msg_type, item) \
  sbp_send_msg(msg_type, sizeof(item), (u8 *)&(item))

/** SBP callback function definition. */
typedef void (*msg_callback_t)(u8 msg[]);

/** SBP callback node.
 * Forms a linked list of callbacks.
 * \note Must be statically allocated for use with sbp_register_callback().
 */
typedef struct msg_callbacks_node {
  u8 msg_type;                      /**< Message ID associated with callback. */
  msg_callback_t cb;                /**< Pointer to callback function. */
  struct msg_callbacks_node *next;  /**< Pointer to next node in list. */
} msg_callbacks_node_t;

/** State structure for processing SBP messages from a particular USART. */
typedef struct {
  enum {
    WAITING_1 = 0,
    WAITING_2,
    GET_TYPE,
    GET_LEN,
    GET_MSG,
    GET_CRC
  } state;
  u8 msg_type;
  u8 msg_len;
  u8 msg_n_read;
  u8 msg_buff[256];
  u8 crc_n_read;
  u8 crc[2];
  usart_rx_dma_state *rx_state;
} sbp_process_messages_state_t;

/** \} */

void sbp_setup(u8 use_settings);
void sbp_disable(void);
u32 sbp_send_msg(u8 msg_type, u8 len, u8 buff[]);
void sbp_register_callback(u8 msg_type, msg_callback_t cb,
                           msg_callbacks_node_t *node);
msg_callback_t sbp_find_callback(u8 msg_id);
void sbp_process_usart(sbp_process_messages_state_t *s);
void sbp_process_messages();

void debug_variable(char *name, double x);

#define DEBUG_VAR(name, x, rate) { \
  DO_EVERY_TICKS(TICK_FREQ/rate,   \
      debug_variable((name), (x)); \
  ); }

#endif

