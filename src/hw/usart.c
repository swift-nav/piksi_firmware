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

#include "../debug.h"
#include "../swift_nap_io.h"
#include "leds.h"
#include "usart.h"

u8 usart_fifo_tx[USART_BUFFER_LEN];
u8 usart_fifo_rx[USART_BUFFER_LEN];
u16 usart_fifo_tx_rd;
u16 usart_fifo_tx_wr;
u16 usart_fifo_rx_rd;
u32 usart_fifo_rx_rd_wraps;
u32 usart_fifo_rx_wr_wraps;
u16 usart_dma_xfer_len;

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
	usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_enable_tx_dma(USART1);
  usart_enable_rx_dma(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);

  /* USART1 TX - DMA2, stream 7, channel 4, low priority*/
  DMA2_S7CR = 0; /* Make sure stream is disabled to start. */
  DMA2_S7CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |  // Error interrupts
              DMA_SxCR_TCIE |                   // Transfer complete interrupt
              DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
              DMA_SxCR_MINC |                   // Increment the memory address after each transfer
              DMA_SxCR_PSIZE_8BIT |             // 8 bit transfers to USART peripheral
              DMA_SxCR_MSIZE_8BIT |             // and from memory
              DMA_SxCR_PL_LOW |                 // Low priority
              DMA_SxCR_CHSEL(4);                // The channel selects which request line will trigger a transfer
                                                // In this case, channel 4 = UART1_TX (see CD00225773.pdf Table 23)

  DMA2_S7NDTR = 0;  // For now, don't transfer any number of datas (will be set in the initiating function)

  DMA2_S7PAR = &USART1_DR;      // DMA into the USART1 data register
  DMA2_S7M0AR = usart_fifo_tx;  // from the usart_fifo_tx.

  // TODO: Investigate more about the best FIFO settings.
  DMA2_S7FCR = DMA_SxFCR_DMDIS |          // Enable DMA stream FIFO
               DMA_SxFCR_FTH_2_4_FULL |   // Triger level 1/2 full
               DMA_SxFCR_FEIE;            // Enable FIFO error interrupt

  usart_fifo_tx_wr = usart_fifo_tx_rd = 0;  // Buffer is empty to begin with.

  /* USART1 RX - DMA2, stream 5, channel 4, low priority*/
  DMA2_S5CR = 0; /* Make sure stream is disabled to start. */
  DMA2_S5CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |  // Error interrupts
              DMA_SxCR_TCIE |                   // Transfer complete interrupt
              DMA_SxCR_CIRC |                   // Enable circular buffer mode
              DMA_SxCR_DIR_PERIPHERAL_TO_MEM |
              DMA_SxCR_MINC |                   // Increment the memory address after each transfer
              DMA_SxCR_PSIZE_8BIT |             // 8 bit transfers to USART peripheral
              DMA_SxCR_MSIZE_8BIT |             // and from memory
              DMA_SxCR_PL_LOW |                 // Low priority
              DMA_SxCR_CHSEL(4);                // The channel selects which request line will trigger a transfer
                                                // In this case, channel 4 = UART1_RX (see CD00225773.pdf Table 23)

  DMA2_S5NDTR = USART_BUFFER_LEN;  // For now, don't transfer any number of datas (will be set in the initiating function)

  DMA2_S5PAR = &USART1_DR;      // DMA into the USART1 data register
  DMA2_S5M0AR = usart_fifo_rx;  // from the usart_fifo_tx.

  // TODO: Investigate more about the best FIFO settings.
  DMA2_S5FCR = DMA_SxFCR_DMDIS |          // Enable DMA stream FIFO
               DMA_SxFCR_FTH_2_4_FULL |   // Triger level 1/2 full
               DMA_SxFCR_FEIE;            // Enable FIFO error interrupt

  usart_fifo_rx_rd = 0;  // Buffer is empty to begin with.
  usart_fifo_rx_rd_wraps = usart_fifo_rx_wr_wraps = 0;

  /* Enable DMA2 Stream 7 (TX) interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);

  /* Enable DMA2 Stream 5 (RX) interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);

  DMA2_S5CR |= DMA_SxCR_EN;
}

void usart_write_dma(u8 data[], u16 n)
{
  u16 wr;

  if (n == 0) return;

  /* Disable interrupts to "atomically" increment
   * usart_fifo_tx_wr as we want this function to be reentrant.
   */
  __asm__("CPSID i;");
  // TODO: Check for buffer overflow
  wr = usart_fifo_tx_wr;
  usart_fifo_tx_wr = (usart_fifo_tx_wr + n) % USART_BUFFER_LEN;
  __asm__("CPSIE i;");

  if (wr + n <= USART_BUFFER_LEN)
    memcpy(&usart_fifo_tx[wr], data, n);
  else {
    /* Deal with case where write wraps the circular buffer. */
    memcpy(&usart_fifo_tx[wr], &data[0], USART_BUFFER_LEN - wr);
    memcpy(&usart_fifo_tx[0], &data[USART_BUFFER_LEN-wr], n - (USART_BUFFER_LEN - wr));
  }

  /* Check if there is a DMA transfer either in progress or waiting
   * for its interrupt to be serviced. Its very important to also
   * check the interrupt flag as EN will be cleared when the transfer
   * finishes but we really need to make sure the ISR has been run to
   * finish up the bookkeeping for the transfer.
   */
  if (!((DMA2_S7CR & DMA_SxCR_EN) || (DMA2_HISR & DMA_HISR_TCIF7)))
    usart_tx_dma_schedule();
}

void usart_tx_dma_schedule()
{
  /* Disable interrupts to "atomically" schedule the DMA. */
  __asm__("CPSID i;");

  DMA2_S7M0AR = &usart_fifo_tx[usart_fifo_tx_rd];

  if (usart_fifo_tx_rd < usart_fifo_tx_wr) {
    /* DMA up until usart_fifo_tx_wr. */
    DMA2_S7NDTR = usart_fifo_tx_wr - usart_fifo_tx_rd;
  } else {
    /* DMA up until the end of the FIFO buffer. */
    DMA2_S7NDTR = USART_BUFFER_LEN - usart_fifo_tx_rd;
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
    usart_fifo_tx_rd = (usart_fifo_tx_rd + usart_dma_xfer_len) % USART_BUFFER_LEN;
    if (usart_fifo_tx_wr != usart_fifo_tx_rd)
      // FIFO not empty.
      usart_tx_dma_schedule();
    __asm__("CPSIE i;");
  } else {
    // TODO: Handle error interrupts! */
    screaming_death();
  }
}

void dma2_stream5_isr()
{
  if (DMA2_HISR & DMA_HISR_TCIF5) {
    /* Interrupt is Transmit Complete. We are in circular buffer mode
     * so this probably means we just wrapped the buffer.
     */

    /* Clear the DMA transmit complete interrupt flag. */
    DMA2_HIFCR = DMA_HIFCR_CTCIF5;

    /* Increment our write wrap counter. */
    usart_fifo_rx_wr_wraps++;
  } else {
    // TODO: Handle error interrupts! */
    screaming_death();
  }
}

u16 usart_n_read_dma()
{
  __asm__("CPSID i;");
  // Compare number of bytes written to the number read, if greater
  // than a whole buffer then we have had an overflow.
  u32 n_read = usart_fifo_rx_rd_wraps*USART_BUFFER_LEN + usart_fifo_rx_rd;
  u32 n_written = (usart_fifo_rx_wr_wraps+1)*USART_BUFFER_LEN - DMA2_S5NDTR;
  __asm__("CPSIE i;");
  if (n_written - n_read > USART_BUFFER_LEN) {
    // My buffer runneth over.
    screaming_death();
  }
  return n_written - n_read;
}

u16 usart_read_dma(u8 buff[], u16 len)
{
  u16 n_to_read = usart_n_read_dma();
  u16 n = (len > n_to_read) ? n_to_read : len;

  if (usart_fifo_rx_rd + n < USART_BUFFER_LEN) { 
    memcpy(buff, &usart_fifo_rx[usart_fifo_rx_rd], n);
    usart_fifo_rx_rd += n;
  } else {
    usart_fifo_rx_rd_wraps++;
    memcpy(&buff[0], &usart_fifo_rx[usart_fifo_rx_rd], USART_BUFFER_LEN - usart_fifo_rx_rd);
    memcpy(&buff[USART_BUFFER_LEN - usart_fifo_rx_rd], &usart_fifo_rx[0], n - USART_BUFFER_LEN + usart_fifo_rx_rd);
    usart_fifo_rx_rd = n - USART_BUFFER_LEN + usart_fifo_rx_rd;
  }

  return n;
}
