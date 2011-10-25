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
#include "m25_flash.h"
#include "board/leds.h"

u8 in_buff[2+255];
u8 in_packet_valid = 0;
u8 in_packet_count = 0;
enum {
  waiting_1,
  waiting_2,
  get_type,
  get_len,
  get_packet
} in_packet_state;

void debug_setup() {
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9|GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200, 16368000);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

  /* Enable USART1 inerrupts wit the NVIC. */
  nvic_enable_irq(NVIC_USART1_IRQ);

  /* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);

  /* Disable input and output bufferings */
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  nvic_set_pending_irq(NVIC_USART1_IRQ);
}

void send_debug_msg(u8 msg_type, u8 len, u8 buff[]) {
  usart_send_blocking(USART1, DEBUG_MAGIC_1);
  usart_send_blocking(USART1, DEBUG_MAGIC_2);
  usart_send_blocking(USART1, msg_type);
  usart_send_blocking(USART1, len);
  while(len--)
    usart_send_blocking(USART1, *buff++);
}

void usart1_isr(void)
{
	static u8 data = 0;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
		((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);
    
    switch (in_packet_state) {
      case waiting_1:
        if (data == DEBUG_MAGIC_1)
          in_packet_state = waiting_2;
        break;
      case waiting_2:
        if (data == DEBUG_MAGIC_2) {
          led_on(LED_GREEN);
          in_packet_state = get_type;
        }
        break;
      case get_type:
        /* TODO: detect if we have an unprocessed packet here 
         * we are overwriting. */
        in_packet_valid = 0;
        in_buff[0] = data;
        in_packet_state = get_len;
        break;
      case get_len:
        in_buff[1] = data;
        in_packet_state = get_packet;
        in_packet_count = 0;
        break;
      case get_packet:
        /* TODO: Setup DMA transfer here. */
        in_buff[2+in_packet_count] = data;
        in_packet_count++;
        if (in_packet_count == in_buff[1]) {
          in_packet_valid = 1;
          in_packet_state = waiting_1;
        } else
          in_packet_state = get_packet;
        break;
    }
	}
}

u8* get_in_packet() {
  /* TODO: this should really be a packet queue. */
  if (!in_packet_valid)
    return 0;
  else {
    in_packet_valid = 0;
    return in_buff;
  }
}

void process_packet() {
  u8 type, length;
  u8* buff;

  buff = get_in_packet();

  if (buff) {
    type = buff[0];
    length = buff[1];
    buff = &buff[2];

    switch(type) {
      case MSG_U32:
        printf("Got u32: 0x%08X\n", *(unsigned int*)buff);
        break;

      case MSG_FLASH_WRITE: {
        msg_flash_write_t* fw = (msg_flash_write_t*)buff;
        printf("Got flash write cmd addr=0x%08X, len=%d\n", 
            (unsigned int)fw->address, (unsigned int)fw->length);
        for (u8 i = 0; i<fw->length; i++) {
          printf("%02X ", fw->data[i]);
          if ((i+1) % 16 == 0)
            printf("\n");
        }
        m25_write_enable();
        m25_page_program(fw->address, fw->length, fw->data);
      }

      case MSG_FLASH_READ: {
        u8 read_buff[250];
        msg_flash_read_t* fr = (msg_flash_read_t*)buff;
        m25_read(fr->address, fr->length, read_buff);
        printf("Got flash read cmd addr=0x%08X, len=%d\n", 
            (unsigned int)fr->address, (unsigned int)fr->length);
        for (u8 i = 0; i<fr->length; i++) {
          printf("%02X ", read_buff[i]);
          if ((i+1) % 16 == 0)
            printf("\n");
        }
        break;
      }

      case MSG_FLASH_ERASE_ALL:
        printf("Got flash bulk erase cmd\n");
        m25_write_enable();
        m25_bulk_erase();
        break;

      default:
        printf("Got unknown packet ID=0x%02X, len=%d, contents:\n", type, length);
        for (u8 i = 0; i<length; i++) {
          printf("%02X ", buff[i]);
          if ((i+1) % 16 == 0)
            printf("\n");
        }
        printf("\n\n");
    }
  }
}

int _write (int file, char *ptr, int len)
{
	if (file == 1) {
    if (len > 255) len = 255; /* Send maximum of 255 chars at a time */

    send_debug_msg(MSG_PRINT, len, (u8*)ptr);
		return len;
	}
  errno = EIO;
  return -1;
}

