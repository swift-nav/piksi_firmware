/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/usart.h>

#include <libswiftnav/edc.h>
#include <libswiftnav/sbp.h>

#include "board/leds.h"
#include "board/m25_flash.h"
#include "error.h"
#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"

/** \defgroup io Input/Output
 * Communications to and from host.
 * \{ */

/** \defgroup sbp Swift Binary Protocol
 * Send and receive messages using Swift Binary Protocol.
 * \{ */

u16 my_sender_id;
u8 sbp_use_settings = 0;

sbp_state_t uarta_sbp_state;
sbp_state_t uartb_sbp_state;
sbp_state_t ftdi_sbp_state;

/** Setup the SBP interface.
 * Configures USARTs and IO buffering.
 *
 * \param use_settings If 0 use default baud rate, else use baud rates in
 *                     flash settings
 */
void sbp_setup(u8 use_settings, u16 sender_id)
{
  my_sender_id = sender_id;

  if (use_settings && settings.settings_valid == VALID) {
    sbp_use_settings = 1;
    usarts_setup(
      settings.ftdi_usart.baud_rate,
      settings.uarta_usart.baud_rate,
      settings.uartb_usart.baud_rate
      );
  } else {
    sbp_use_settings = 0;
    usarts_setup(
      USART_DEFAULT_BAUD,
      USART_DEFAULT_BAUD,
      USART_DEFAULT_BAUD
      );
  }

  sbp_state_init(&uarta_sbp_state);
  sbp_state_init(&uartb_sbp_state);
  sbp_state_init(&ftdi_sbp_state);

  /* Disable input and output buffering. */
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
}

/** Disable the SBP interface.
 * Disables the USART peripherals and DMA streams enabled by sbp_setup(). */
void sbp_disable()
{
  usarts_disable();
}

/** Checks if the message should be sent from a particular USART. */
static inline u32 use_usart(usart_settings_t *us, u16 msg_type)
{
  if (sbp_use_settings) {
    if (us->mode != SBP)
      /* This USART is not in SBP mode. */
      return 0;

    if (!(us->message_mask & msg_type))
      /* This message type is masked out on this USART. */
      return 0;
  }
  return 1;
}

u32 uarta_write(u8 *buff, u32 n)
{
  return usart_write_dma(&uarta_tx_state, buff, n);
}
u32 uartb_write(u8 *buff, u32 n)
{
  return usart_write_dma(&uartb_tx_state, buff, n);
}
u32 ftdi_write(u8 *buff, u32 n)
{
  return usart_write_dma(&ftdi_tx_state, buff, n);
}

/** Send a SBP message out over all applicable USARTs
 *
 * \param msg_type Message ID (defined in sbp_messages.h)
 * \param len      Length of message data
 * \param buff     Pointer to message data array
 *
 * \return         Error code
 */
u32 sbp_send_msg(u16 msg_type, u8 len, u8 buff[])
{
  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  u16 ret = 0;

  if (use_usart(&settings.uarta_usart, msg_type))
    ret |= sbp_send_message(msg_type, my_sender_id, len, buff, &uarta_write);

  if (use_usart(&settings.uartb_usart, msg_type))
    ret |= sbp_send_message(msg_type, my_sender_id, len, buff, &uarta_write);

  if (use_usart(&settings.ftdi_usart, msg_type))
    ret |= sbp_send_message(msg_type, my_sender_id, len, buff, &ftdi_write);

  if (ret != 3*len) {
    /* Return error if any sbp_send_message failed. */
    __asm__("CPSIE i;");  /* Re-enable interrupts */
    return ret;
  }

  __asm__("CPSIE i;");  // Re-enable interrupts
  return 0;
}

u32 uarta_read(u8 *buff, u32 n)
{
  return usart_read_dma(&uarta_rx_state, buff, n);
}
u32 uartb_read(u8 *buff, u32 n)
{
  return usart_read_dma(&uartb_rx_state, buff, n);
}
u32 ftdi_read(u8 *buff, u32 n)
{
  return usart_read_dma(&ftdi_rx_state, buff, n);
}

/** Process SBP messages received through the USARTs.
 * This function should be called periodically to clear the USART DMA RX
 * buffers and handle the SBP callbacks in them.
 */
void sbp_process_messages()
{
  while (usart_n_read_dma(&uarta_rx_state) > 0) {
    sbp_process(&uarta_sbp_state, &uarta_read);
  }
  while (usart_n_read_dma(&uartb_rx_state) > 0) {
    sbp_process(&uartb_sbp_state, &uartb_read);
  }
  while (usart_n_read_dma(&ftdi_rx_state) > 0) {
    sbp_process(&ftdi_sbp_state, &ftdi_read);
  }
}

/** Directs printf's output to the SBP interface */
int _write(int file, char *ptr, int len)
{
  switch (file) {
  case 1:
    if (len > 255) len = 255;   /* Send maximum of 255 chars at a time */
    sbp_send_msg(MSG_PRINT, len, (u8 *)ptr);
    return len;

  case 22:
    if (len > 255) len = 255;   /* Send maximum of 255 chars at a time */
    usart_write_dma(&ftdi_tx_state, (u8 *)ptr, len);
    return len;

  default:
    errno = EIO;
    return -1;
  }
}

void debug_variable(char *name, double x)
{
  u8 sl = strlen(name);
  u8* buff = malloc(sl + sizeof(double));
  memcpy(buff, &x, sizeof(double));
  memcpy(&buff[8], name, sl);
  sbp_send_msg(MSG_DEBUG_VAR, sl + sizeof(double), buff);
  free(buff);
}

/** \} */

/** \} */

