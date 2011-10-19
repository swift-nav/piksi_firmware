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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <stdio.h>
#include <errno.h>

#include "debug.h"

void debug_setup() {
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9|GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200, 16000000);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void send_debug_msg(u8 msg_type, u8 len, u8 buff[]) {
  usart_send_blocking(USART1, DEBUG_MAGIC_1);
  usart_send_blocking(USART1, DEBUG_MAGIC_2);
  usart_send_blocking(USART1, msg_type);
  usart_send_blocking(USART1, len);
  while(len--)
    usart_send_blocking(USART1, *buff++);
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

