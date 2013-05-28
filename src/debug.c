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

/* Store a pointer to the head of our linked list. */
msg_callbacks_node_t* msg_callbacks_head = 0;

u8 debug_use_settings = 0;

/* CRC16 implementation acording to CCITT standards */
static const u16 crc16tab[256] = {
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

u16 crc16_ccitt(const u8* buf, u8 len, u16 crc)
{
  for (u8 i=0; i<len; i++)
    crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
  return crc;
}

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
      USART_DEFAULT_BAUD,
      USART_DEFAULT_BAUD,
      USART_DEFAULT_BAUD
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

/** Checks if the message should be sent from a particular USART. */
static inline u32 use_usart(usart_settings_t* us, u8 msg_type)
{
  if (debug_use_settings) {
    if (us->mode != PIKSI_BINARY)
      /* This USART is not in Piksi Binary mode. */
      return 0;
    if (!(us->message_mask & msg_type))
      /* This message type is masked out on this USART. */
      return 0;
  }
  return 1;
}

/** Check if USART has room in its buffer. */
static inline u32 check_usart(usart_settings_t* us, usart_tx_dma_state* s,
                              u8 msg_type, u8 len)
{
  return (use_usart(us, msg_type) && usart_tx_n_free(s) < len + 4U + 2U + 1U);
}

static inline u32 send_msg_helper(usart_settings_t* us, usart_tx_dma_state* s,
                                  u8 msg_type, u8 len, u8 buff[], u16 crc)
{
  if (!use_usart(us, msg_type))
    return 0;

  if ((4 != usart_write_dma(s, msg_header, 4)) ||
      (len != usart_write_dma(s, buff, len)) ||
      (2 != usart_write_dma(s, (u8*)&crc, 2))) {
    /* Error during USART write. */
    return -1;
  }
  return 0;
}


u32 debug_send_msg(u8 msg_type, u8 len, u8 buff[])
{
  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

    msg_header[2] = msg_type;
    msg_header[3] = len;

    u16 crc = crc16_ccitt(&msg_header[2], 2, 0);
    crc = crc16_ccitt(buff, len, crc);

    /* Check if required USARTs have room in their buffers. */
    if (check_usart(&settings.ftdi_usart, &ftdi_tx_state, len, msg_type) ||
        check_usart(&settings.uarta_usart, &uarta_tx_state, len, msg_type) ||
        check_usart(&settings.uartb_usart, &uartb_tx_state, len, msg_type)) {
      __asm__("CPSIE i;");  // Re-enable interrupts
      return -1;
    }

    /* Now send message. */
    if (send_msg_helper(&settings.ftdi_usart, &ftdi_tx_state,
                        msg_type, len, buff, crc) ||
        send_msg_helper(&settings.uarta_usart, &uarta_tx_state,
                        msg_type, len, buff, crc) ||
        send_msg_helper(&settings.uartb_usart, &uartb_tx_state,
                        msg_type, len, buff, crc)) {
      __asm__("CPSIE i;");  // Re-enable interrupts
      return -1;
    }

  __asm__("CPSIE i;");  // Re-enable interrupts
  return 0;
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
  u16 crc, crc_rx;

  while((len = usart_n_read_dma(s->rx_state)))
  {
    /* If there are no bytes waiting to be processed then return. */
    if (len == 0)
      return;

    switch(s->state) {
      case WAITING_1:
        if (usart_read_dma(s->rx_state, &temp, 1)) {
          if (temp == DEBUG_MAGIC_1)
            s->state = WAITING_2;
        }
        break;
      case WAITING_2:
        if (usart_read_dma(s->rx_state, &temp, 1)) {
          if (temp == DEBUG_MAGIC_2)
            s->state = GET_TYPE;
          else
            s->state = WAITING_1;
        }
        break;
      case GET_TYPE:
        if (usart_read_dma(s->rx_state, &(s->msg_type), 1))
          s->state = GET_LEN;
        break;
      case GET_LEN:
        if (usart_read_dma(s->rx_state, &(s->msg_len), 1)) {
          s->msg_n_read = 0;
          s->state = GET_MSG;
        }
        break;
      case GET_MSG:
        if (s->msg_len - s->msg_n_read > 0) {
          /* Not received whole message yet, try and get some more. */
          s->msg_n_read += usart_read_dma(
              s->rx_state,
              &(s->msg_buff[s->msg_n_read]),
              s->msg_len - s->msg_n_read
          );
        }
        /*
         * TODO : <= ? change to == and have a separate case for < ?
         * CRC should catch this though.
         */
        if (s->msg_len - s->msg_n_read <= 0) {
          s->crc_n_read = 0;
          s->state = GET_CRC;
        }
        break;
      case GET_CRC:
        if (s->crc_n_read < 2) {
          s->crc_n_read += usart_read_dma(
              s->rx_state,
              &(s->crc[s->crc_n_read]),
              2 - s->crc_n_read
          );
        }
        if (s->crc_n_read >= 2) {
          crc = crc16_ccitt(&(s->msg_type), 1, 0);
          crc = crc16_ccitt(&(s->msg_len), 1, crc);
          crc = crc16_ccitt(s->msg_buff, s->msg_len, crc);
          crc_rx = (s->crc[0]) |
                  ((s->crc[1] & 0xFF) << 8);
          if (crc_rx == crc) {
            /* Message complete, process it. */
            msg_callback_t cb = debug_find_callback(s->msg_type);
            if (cb)
              (*cb)(s->msg_buff);
            else
              printf("no callback registered for msg type %02X\n", s->msg_type);
          } else {
            printf("CRC error 0x%04X 0x%04X\n", crc, crc_rx);
          }
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

