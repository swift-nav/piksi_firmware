/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/dma.h>

#include "error.h"
#include "settings.h"
#include "debug.h"
#include "hw/m25_flash.h"
#include "hw/leds.h"
#include "hw/usart.h"

u8 msg_header[4] = {DEBUG_MAGIC_1, DEBUG_MAGIC_2, 0, 0};

u8 msg_buff[256];

/* Store a pointer to the head of our linked list. */
msg_callbacks_node_t* msg_callbacks_head = 0;

u8 debug_use_settings = 0;

void debug_setup(u8 use_settings)
{
  if (use_settings && settings.settings_valid == VALID) {
    debug_use_settings = 1;
    usarts_setup(
      settings.ftdi_usart.baud_rate,
      settings.uarta_usart.baud_rate,
      settings.uartb_usart.baud_rate
    );
  } else {
    debug_use_settings = 0;
    usarts_setup(
      USART_DEFUALT_BAUD,
      USART_DEFUALT_BAUD,
      USART_DEFUALT_BAUD
    );
  }

  /* Disable input and output buffering. */
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
}

/** Disables the USART peripherals and DMA streams enabled by debug_setup(). */
void debug_disable()
{
  usarts_disable();
}

u32 debug_send_msg(u8 msg_type, u8 len, u8 buff[])
{
  /* Global interrupt disable to avoid concurrency/reentrance problems. */
  __asm__("CPSID i;");

    msg_header[2] = msg_type;
    msg_header[3] = len;

    if (debug_use_settings ||
        (settings.ftdi_usart.mode == PIKSI_BINARY &&
         settings.ftdi_usart.message_mask & msg_type)) {
      if (4 != usart_write_dma(&ftdi_tx_state, msg_header, 4))
        speaking_death("debug_send_message failed on FTDI");
      if (len != usart_write_dma(&ftdi_tx_state, buff, len))
        speaking_death("debug_send_message failed on FTDI");
    }

    if (debug_use_settings ||
        (settings.uarta_usart.mode == PIKSI_BINARY &&
         settings.uarta_usart.message_mask & msg_type)) {
      if (4 != usart_write_dma(&uarta_tx_state, msg_header, 4))
        speaking_death("debug_send_message failed on UARTA");
      if (len != usart_write_dma(&uarta_tx_state, buff, len))
        speaking_death("debug_send_message failed on UARTA");
    }

    if (debug_use_settings ||
        (settings.uartb_usart.mode == PIKSI_BINARY &&
         settings.uartb_usart.message_mask & msg_type)) {
      if (4 != usart_write_dma(&uartb_tx_state, msg_header, 4))
        speaking_death("debug_send_message failed on UARTB");
      if (len != usart_write_dma(&uartb_tx_state, buff, len))
        speaking_death("debug_send_message failed on UARTB");
    }

  __asm__("CPSIE i;");  // Re-enable interrupts
  return 0; // Successfully written to buffer.
}

/** Register a callback for a message type.
 * Register a callback that is called when a message
 * with type msg_type is received.
 *
 * This function must be passed a pointer to a
 * STATICALLY ALLOCATED msg_callback_node_t which it
 * will use to store a reference to your callback.
 * This struct need not be initialised.
 */
void debug_register_callback(u8 msg_type, msg_callback_t cb, msg_callbacks_node_t* node)
{
  /* Fill in our new msg_callback_node_t. */
  node->msg_type = msg_type;
  node->cb = cb;
  /* The next pointer is set to NULL, i.e. this
   * will be the new end of the linked list.
   */
  node->next = 0;

  /* If our linked list is empty then just
   * add the new node to the start.
   */
  if (msg_callbacks_head == 0) {
    msg_callbacks_head = node;
    return;
  }

  /* Find the tail of our linked list and
   * add our new node to the end.
   */
  msg_callbacks_node_t* p = msg_callbacks_head;
  while(p->next)
    p = p->next;

  p->next = node;
}

msg_callback_t debug_find_callback(u8 msg_type)
{
  /* If our list is empty, return NULL. */
  if (!msg_callbacks_head)
    return 0;

  /* Traverse the linked list and return the callback
   * function pointer if we find a node with a matching
   * message id.
   */
  msg_callbacks_node_t* p = msg_callbacks_head;
  do {
    if (p->msg_type == msg_type)
      return p->cb;
  } while((p = p->next));

  /* Didn't find a matching callback, return NULL. */
  return 0;
}

void debug_process_messages()
{
  static debug_process_messages_state_t ftdi_s = {
    .state = WAITING_1,
    .rx_state = &ftdi_rx_state,
  };
  static debug_process_messages_state_t uarta_s = {
    .state = WAITING_1,
    .rx_state = &uarta_rx_state,
  };
  static debug_process_messages_state_t uartb_s = {
    .state = WAITING_1,
    .rx_state = &uartb_rx_state,
  };

  debug_process_usart(&ftdi_s);
  debug_process_usart(&uarta_s);
  debug_process_usart(&uartb_s);
}

void debug_process_usart(debug_process_messages_state_t* s)
{
  u8 len, temp;

  while((len = usart_n_read_dma(s->rx_state)))
  {
    /* If there are no bytes waiting to be processed then return. */
    if (len == 0)
      return;

    switch(s->state) {
      case WAITING_1:
        usart_read_dma(s->rx_state, &temp, 1);
        if (temp == DEBUG_MAGIC_1)
          s->state = WAITING_2;
        break;
      case WAITING_2:
        usart_read_dma(s->rx_state, &temp, 1);
        if (temp == DEBUG_MAGIC_2)
          s->state = GET_TYPE;
        break;
      case GET_TYPE:
        usart_read_dma(s->rx_state, &(s->msg_type), 1);
        s->state = GET_LEN;
        break;
      case GET_LEN:
        usart_read_dma(s->rx_state, &(s->msg_len), 1);
        s->msg_n_read = 0;
        s->state = GET_MSG;
        break;
      case GET_MSG:
        if (s->msg_len - s->msg_n_read > 0) {
          /* Not received whole message yet, try and get some more. */
          s->msg_n_read += usart_read_dma(
              s->rx_state,
              &msg_buff[s->msg_n_read],
              s->msg_len - s->msg_n_read
          );
        }
        if (s->msg_len - s->msg_n_read <= 0) {
          /* Message complete, process it. */
          /*printf("msg: %02X, len %d\n", s->msg_type, s->msg_len);*/
          msg_callback_t cb = debug_find_callback(s->msg_type);
          if (cb)
            (*cb)(msg_buff);
          else
            printf("no callback registered for msg type %02X\n", s->msg_type);
          s->state = WAITING_1;
        }
        break;
      default:
        s->state = WAITING_1;
        break;
    }
  }
}

int _write (int file, char *ptr, int len)
{
	switch (file) {
    case 1:
      if (len > 255) len = 255; /* Send maximum of 255 chars at a time */
      debug_send_msg(MSG_PRINT, len, (u8*)ptr);
      return len;

    case 22:
      if (len > 255) len = 255; /* Send maximum of 255 chars at a time */
      usart_write_dma(&ftdi_tx_state, (u8*)ptr, len);
      return len;

    default:
      errno = EIO;
      return -1;
	}
}

