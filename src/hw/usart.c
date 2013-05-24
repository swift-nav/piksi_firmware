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
#include <libopencm3/stm32/f4/dma.h>

#include "../settings.h"
#include "usart.h"

usart_rx_dma_state ftdi_rx_state;
usart_tx_dma_state ftdi_tx_state;
usart_rx_dma_state uarta_rx_state;
usart_tx_dma_state uarta_tx_state;
usart_rx_dma_state uartb_rx_state;
usart_tx_dma_state uartb_tx_state;

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
 * USART 6, 1 and 3 peripherals are configured
 * (connected to the FTDI, UARTA and UARTB ports on the Piksi respectively).
 */
void usarts_setup(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud)
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

  usart_set_parameters(USART6, ftdi_baud);
  usart_set_parameters(USART1, uarta_baud);
  usart_set_parameters(USART3, uartb_baud);

  /* FTDI (USART6) TX - DMA2, stream 6, channel 5. */
  usart_tx_dma_setup(&ftdi_tx_state, USART6, DMA2, 6, 5);
  /* FTDI (USART6) RX - DMA2, stream 1, channel 5. */
  usart_rx_dma_setup(&ftdi_rx_state, USART6, DMA2, 1, 5);

  /* UARTA (USART1) TX - DMA2, stream 7, channel 4. */
  usart_tx_dma_setup(&uarta_tx_state, USART1, DMA2, 7, 4);
  /* UARTA (USART1) RX - DMA2, stream 2, channel 4. */
  usart_rx_dma_setup(&uarta_rx_state, USART1, DMA2, 2, 4);

  /* UARTB (USART3) TX - DMA1, stream 3, channel 4. */
  usart_tx_dma_setup(&uartb_tx_state, USART3, DMA1, 3, 4);
  /* UARTB (USART3) RX - DMA1, stream 1, channel 4. */
  usart_rx_dma_setup(&uartb_rx_state, USART3, DMA1, 1, 4);
}

void usarts_disable()
{
  /* Disable DMA channels. */
  usart_tx_dma_disable(&ftdi_tx_state);
  usart_rx_dma_disable(&ftdi_rx_state);
  usart_tx_dma_disable(&uarta_tx_state);
  usart_rx_dma_disable(&uarta_rx_state);
  usart_tx_dma_disable(&uartb_tx_state);
  usart_rx_dma_disable(&uartb_rx_state);

  /* Disable all USARTs. */
  usart_disable(USART6);
  usart_disable(USART1);
  usart_disable(USART3);
}

void dma2_stream6_isr(void)
{
  usart_tx_dma_isr(&ftdi_tx_state);
}
void dma2_stream1_isr(void)
{
  usart_rx_dma_isr(&ftdi_rx_state);
}
void dma2_stream7_isr(void)
{
  usart_tx_dma_isr(&uarta_tx_state);
}
void dma2_stream2_isr(void)
{
  usart_rx_dma_isr(&uarta_rx_state);
}
void dma1_stream3_isr(void)
{
  usart_tx_dma_isr(&uartb_tx_state);
}
void dma1_stream1_isr(void)
{
  usart_rx_dma_isr(&uartb_rx_state);
}

void usart_disable_common(void){
  usart_disable(USART6);
}
