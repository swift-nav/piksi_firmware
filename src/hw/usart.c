/*
 * Copyright (C) 2012 Fergus Noble <fergusnoble@gmail.com>
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

#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/usart.h>

#include "usart.h"

/** Setup the USART peripheral. */
void usart_setup_common(void) {
  /* Set up the USART6 peripheral. */

  /* First give everything a clock. */
  /* Clock the USART. */
  RCC_APB2ENR |= RCC_APB2ENR_USART6EN;
  /* GPIO pins corresponding to the USART. */
	RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;

  /* Assign the GPIO pins appropriately. */
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
	gpio_set_af(GPIOC, GPIO_AF8, GPIO6|GPIO7);

  /* Setup UART parameters. */
  usart_disable(USART6);
	usart_set_baudrate(USART6, 230400);
	usart_set_databits(USART6, 8);
	usart_set_stopbits(USART6, USART_STOPBITS_1);
	usart_set_parity(USART6, USART_PARITY_NONE);
	usart_set_flow_control(USART6, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART6, USART_MODE_TX_RX);

	/* Enable the USART. */
	usart_enable(USART6);
}

void usart_disable_common(void){
  usart_disable(USART6);
}
