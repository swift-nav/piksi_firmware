/*
 * Copyright (C) 2012 Henry Hallam <henry@swift-nav.com>
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

#include <string.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f2/dma.h>

#include "usart.h"

u8 usart_fifo[USART_BUFFER_LEN];
u8 usart_fifo_rd;
u8 usart_fifo_wr;
u8 usart_dma_xfer_len;

void usart_tx_dma_schedule();

void usart_dma_setup(void) {
  // Set up the USART1 peripheral, the DMA and the interrupts

  // First give everything a clock
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;    // Enable clock to DMA peripheral
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN;  // Clock the USART
	RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;    // Don't forget to clock the GPIO pins corresponding to the USART

  // Assign the GPIO pins appropriately
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9|GPIO10);

  /* Setup UART parameters. */
  /*usart_disable(USART1);*/
	usart_set_baudrate(USART1, 921600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);
  usart_enable_tx_dma(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);

  /* USART1 TX - DMA2, stream 7, channel 4, low priority*/
  DMA2_S7CR = 0; /* Make sure stream is disabled to start. */
  DMA2_S7CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |  // Error interrupts
              DMA_SxCR_TCIE |                      // Transfer complete interrupt
              DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
              DMA_SxCR_MINC |                         // Increment the memory address after each transfer
              DMA_SxCR_PSIZE_8BIT |                   // 8 bit transfers to USART peripheral
              DMA_SxCR_MSIZE_8BIT |                   // and from memory
              DMA_SxCR_PL_LOW |                      // Low priority
              DMA_SxCR_CHSEL(4);                      // The channel selects which request line will trigger a transfer
                                                      // In this case, channel 4 = UART1_TX (see CD00225773.pdf Table 23)

  DMA2_S7NDTR = 0;        // For now, don't transfer any number of datas (will be set in the initiating function)

  DMA2_S7PAR = &USART1_DR;    // DMA into the USART1 data register
  DMA2_S7M0AR = usart_fifo;   // from the usart_fifo.

  DMA2_S7FCR = 0;         // DMA FIFO disabled, i.e. direct mode.  TODO: see if FIFO helps performance

  usart_fifo_wr = usart_fifo_rd = 0;  // Buffer is empty to begin with.

  /* Enable DMA2 Stream 7 interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);
}

void usart_write_dma(u8 *data, u8 n)
{
  u8 wr;

  /* Disable interrupts to "atomically" increment
   * usart_fifo_wr as we want this function to be reentrant.
   */
  __asm__("CPSID i;");
  // TODO: Check for buffer overflow
  wr = usart_fifo_wr;
  usart_fifo_wr = (usart_fifo_wr + n) % USART_BUFFER_LEN;
  __asm__("CPSIE i;");

  if (wr + n <= USART_BUFFER_LEN)
    memcpy(&usart_fifo[wr], data, n);
  else {
    /* Deal with case where write wraps the circular buffer. */
    memcpy(&usart_fifo[wr], &data[0], USART_BUFFER_LEN - wr);
    memcpy(&usart_fifo[0], &data[USART_BUFFER_LEN-wr], n - (USART_BUFFER_LEN - wr));
  }

  if (!(DMA2_S7CR & DMA_SxCR_EN))
    usart_tx_dma_schedule();
}

void usart_tx_dma_schedule()
{
  /* Disable interrupts to "atomically" schedule the DMA. */
  __asm__("CPSID i;");

  DMA2_S7M0AR = &usart_fifo[usart_fifo_rd];

  if (usart_fifo_rd < usart_fifo_wr) {
    /* DMA up until usart_fifo_wr. */
    DMA2_S7NDTR = usart_fifo_wr - usart_fifo_rd;
  } else {
    /* DMA up until the end of the FIFO buffer. */
    DMA2_S7NDTR = USART_BUFFER_LEN - usart_fifo_rd;
  }

  /* Save the transfer length so we can increment the read index
   * after the transfer is finished.
   */
  usart_dma_xfer_len = DMA2_S7NDTR;
  /* Enable DMA stream to start transfer. */
  DMA2_S7CR |= DMA_SxCR_EN;

  __asm__("CPSIE i;");
}

void dma2_stream7_isr()
{
  if (DMA2_HISR & DMA_HISR_TCIF7) {
    /* Interrupt is Transmit Complete. */

    /* Clear the DMA transmit complete interrupt flag. */
    DMA2_HIFCR = DMA_HIFCR_CTCIF7;

    __asm__("CPSID i;");
    /* Now that the transfer has finished we can increment the read index. */
    usart_fifo_rd = (usart_fifo_rd + usart_dma_xfer_len) % USART_BUFFER_LEN;
    if (usart_fifo_wr != usart_fifo_rd)
      // FIFO not empty.
      usart_tx_dma_schedule();
    __asm__("CPSIE i;");
  } else {
    // TODO: Handle error interrupts! */
  }
}
