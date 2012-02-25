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

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <stdio.h>
#include <errno.h>

#include "debug.h"
#include "hw/m25_flash.h"
#include "hw/leds.h"
#include "hw/usart.h"

u8 msg_header[4] = {DEBUG_MAGIC_1, DEBUG_MAGIC_2, 0, 0};

u8 msg_buff[USART_BUFFER_LEN];

typedef enum {
  WAITING_1,
  WAITING_2,
  GET_TYPE,
  GET_LEN,
  GET_MSG
} debug_process_messages_state_t;

/* Store a pointer to the head of our linked list. */
msg_callbacks_node_t* msg_callbacks_head = 0;

void debug_setup()
{
  usart_dma_setup();

  /* Disable input and output bufferings */
  /*setvbuf(stdin, NULL, _IONBF, 0);*/
  /*setvbuf(stdout, NULL, _IONBF, 0);*/
}

void debug_send_msg(u8 msg_type, u8 len, u8 buff[])
{
  msg_header[2] = msg_type;
  msg_header[3] = len;
  /* NOTE: these two writes should really be atomic but
   * it doesn't matter too much for debug purposes.
   */
  usart_write_dma(msg_header, 4);
  usart_write_dma(buff, len);
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
  u8 len, temp;
  static u8 msg_type, msg_len, msg_n_read;
  static debug_process_messages_state_t state = WAITING_1;

  while((len = usart_n_read_dma()))
  {
    /* If there are no bytes waiting to be processed then return. */
    if (len == 0)
      return;

    switch(state) {
      case WAITING_1:
        usart_read_dma(&temp, 1);
        if (temp == DEBUG_MAGIC_1)
          state = WAITING_2;
        break;
      case WAITING_2:
        usart_read_dma(&temp, 1);
        if (temp == DEBUG_MAGIC_2)
          state = GET_TYPE;
        break;
      case GET_TYPE:
        usart_read_dma(&msg_type, 1);
        state = GET_LEN;
        break;
      case GET_LEN:
        usart_read_dma(&msg_len, 1);
        msg_n_read = 0;
        state = GET_MSG;
        break;
      case GET_MSG:
        if (msg_len - msg_n_read > 0) {
          /* Not received whole message yet, try and get some more. */
          msg_n_read += usart_read_dma(&msg_buff[msg_n_read], msg_len - msg_n_read);
        }
        if (msg_len - msg_n_read <= 0) {
          /* Message complete, process it. */
          printf("msg: %02X, len %d\n", msg_type, msg_len);
          msg_callback_t cb = debug_find_callback(msg_type);
          if (cb)
            (*cb)(msg_buff);
          else
            printf("no callback registered for msg type %02X\n", msg_type);
          state = WAITING_1;
        }
        break;
      default:
        state = WAITING_1;
        break;
    }
  }
}

int _write (int file, char *ptr, int len)
{
	if (file == 1) {
    if (len > 255) len = 255; /* Send maximum of 255 chars at a time */

    debug_send_msg(MSG_PRINT, len, (u8*)ptr);
		return len;
	}
  errno = EIO;
  return -1;
}

void screaming_death() {
  //disable all interrupts
  __asm__("CPSID f;");
 // __disable_irq();
 
  while(1)
    usart_send_blocking(USART1, '!');
};

