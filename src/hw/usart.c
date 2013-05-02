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

#include "../settings.h"
#include "usart.h"

void usart_set_parameters(u32 usart, u32 baud)
{
  /* Setup UART parameters. */
  baud = baud;
  usart_disable(usart);
	usart_set_baudrate(usart, baud);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart, USART_MODE_TX_RX);

	/* Enable the USART. */
	usart_enable(usart);
}

/** Set up the USART peripherals.
 * USART 6, 1 and 3 peripherals are configured.
 * (connected to the FTDI, UARTA and UARTB ports on the Piksi respectively).
 */
void usarts_setup()
{
  /* First give everything a clock. */

  /* Clock the USARTs. */
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN;
  RCC_APB1ENR |= RCC_APB1ENR_USART3EN;

  /* GPIO pins corresponding to the USART. */
	RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPCEN;

  /* Assign the GPIO pins appropriately:
   *
   * USART   TX    RX  Connection
   * ----------------------------
   * 6      PC6   PC7  FTDI
   * 1      PA9  PA10  UARTA
   * 3     PC10  PC11  UARTB
   */

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
	gpio_set_af(GPIOC, GPIO_AF8, GPIO6|GPIO7);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO9|GPIO10);

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO10|GPIO11);

  usart_set_parameters(USART6, settings.ftdi_usart.baud_rate);
  usart_set_parameters(USART1, settings.uarta_usart.baud_rate);
  usart_set_parameters(USART3, settings.uartb_usart.baud_rate);
}

