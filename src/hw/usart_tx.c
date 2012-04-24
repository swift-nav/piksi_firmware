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

#include "../error.h"
#include "usart.h"

static u8 buff[USART_TX_BUFFER_LEN];
static u32 rd = 0;
static u32 wr = 0;
static u32 xfer_len = 0;

/** Setup the USART for transmission with DMA. This function sets up the DMA
 * controller and additional USART parameters for DMA transmission. The USART
 * must first be configured using \ref usart_setup_common. */
void usart_tx_dma_setup(void) {
  /* Set up the USART1 TX DMA and interrupts. */

  /* Enable clock to DMA peripheral. */
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Enable TX DMA on the USART. */
  usart_enable_tx_dma(USART1);

  /* USART1 TX - DMA2, stream 7, channel 4 */

  /* Make sure stream is disabled to start. */
  DMA2_S7CR &= ~DMA_SxCR_EN;
  DMA2_S7CR = 0;

  /* Configure the DMA controller. */
              /* Error interrupts. */
  DMA2_S7CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
              /* Transfer complete interrupt. */
              DMA_SxCR_TCIE |
              DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
              /* Increment the memory address after each transfer. */
              DMA_SxCR_MINC |
              /* 8 bit transfers to USART peripheral. */
              DMA_SxCR_PSIZE_8BIT |
              /* and from memory. */
              DMA_SxCR_MSIZE_8BIT |
              /* Low priority. */
              DMA_SxCR_PL_LOW |
              /* The channel selects which request line will trigger a
               * transfer. In this case, channel 4 = UART1_TX
               * (see CD00225773.pdf Table 23). */
              DMA_SxCR_CHSEL(4);

  /* For now, don't transfer any number of datas (will be set in the initiating
   * function). */
  DMA2_S7NDTR = 0;

  DMA2_S7PAR = &USART1_DR; /* DMA into the USART1 data register. */
  DMA2_S7M0AR = buff;      /* from the buff. */

  /* TODO: Investigate more about the best FIFO settings. */
  DMA2_S7FCR = DMA_SxFCR_DMDIS |         /* Enable DMA stream FIFO. */
               DMA_SxFCR_FTH_2_4_FULL |  /* Trigger level 1/2 full. */
               DMA_SxFCR_FEIE;           /* Enable FIFO error interrupt. */

  wr = rd = 0;  /* Buffer is empty to begin with. */

  /* Enable DMA2 Stream 7 (TX) interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);
}

/** Disable the USART TX DMA interrupt. Used to ensure that operations on the
 * TX buffer happen "atomically". */
/*static void usart_tx_di(void) {*/
  /*[> TODO: Change this so it doesn't just globally disable interrupts. <]*/
  /*nvic_disable_irq(NVIC_DMA2_STREAM7_IRQ);*/
/*}*/

/** Enable the USART TX DMA interrupt. */
/*static void usart_tx_ei(void) {*/
  /*nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);*/
/*}*/

/** Calculate the space left in the TX buffer.
 *
 * \return The number of bytes that may be safely written to the buffer.
 */
u32 usart_tx_n_free(void) {

  /* The calculation for the number of bytes in the buffer depends on whether
   * or not the write pointer has wrapped around the end of the buffer. */
  if (wr >= rd)
    return USART_TX_BUFFER_LEN - 1 - (wr - rd);
  else
    return (rd - wr) - 1;
}

/** Helper function that schedules a new transfer with the DMA controller if
 * needed. */
static void dma_schedule() {
  /* TODO: We shouldn't have to check for this now that we are called
   * atomically but leaving it in for now just in case. */
  if (DMA2_S7CR & DMA_SxCR_EN)
    speaking_death("DMA TX scheduled while DMA channel running");

  DMA2_S7M0AR = &buff[rd];

  /* Save the transfer length so we can increment the read index after the
   * transfer is finished. */
  if (rd < wr) {
    /* DMA up until write pointer. */
    xfer_len = wr - rd;
  } else {
    /* DMA up until the end of the buffer. */
    xfer_len = USART_TX_BUFFER_LEN - rd;
  }

  /* Set the number of datas in the DMA controller. */
  DMA2_S7NDTR = xfer_len;

  USART1_SR &= ~USART_SR_TC;

  /* Enable DMA stream to start transfer. */
  DMA2_S7CR |= DMA_SxCR_EN;
}

/** USART1 TX DMA interrupt handler. */
void dma2_stream7_isr() {
  if (DMA2_HISR & DMA_HISR_TCIF7) {
    /* Interrupt is Transmit Complete. */

    /* Clear the DMA transmit half and complete interrupt flags. */
    DMA2_HIFCR = DMA_HIFCR_CTCIF7;
    DMA2_HIFCR = DMA_HIFCR_CHTIF7;

    /* Now that the transfer has finished we can increment the read index. */
    rd = (rd + xfer_len) % USART_TX_BUFFER_LEN;

    if (wr != rd)
      /* FIFO not empty. */
      dma_schedule();
  } else {
    /* TODO: Handle error interrupts! */
    speaking_death("DMA TX error interrupt");
  }
}

/** Write out data over the USART using DMA.
 *
 * \param data A pointer to the data to write out.
 * \param n    The number of bytes to write.
 *
 * \return The number of bytes that will be written, may be less than n.
 */
u32 usart_write_dma(u8 data[], u32 n) {
  /* Check if the write would cause a buffer overflow, if so only write up to
   * the end of the buffer. */
  u32 n_free = usart_tx_n_free();
  if (n > n_free)
    n = n_free;

  /* If there is no data to write, just return. */
  if (n == 0) return 0;

  u32 old_wr = wr;
  wr = (wr + n) % USART_TX_BUFFER_LEN;

  if (old_wr + n <= USART_TX_BUFFER_LEN) {
    memcpy(&buff[old_wr], data, n);
  } else {
    /* Deal with case where write wraps the buffer. */
    memcpy(&buff[old_wr], &data[0], USART_TX_BUFFER_LEN - old_wr);
    memcpy(&buff[0], &data[USART_TX_BUFFER_LEN-old_wr],
           n - (USART_TX_BUFFER_LEN - old_wr));
  }

  /* Check if there is a DMA transfer either in progress or waiting for its
   * interrupt to be serviced. Its very important to also check the interrupt
   * flag as EN will be cleared when the transfer finishes but we really need
   * to make sure the ISR has been run to finish up the bookkeeping for the
   * transfer. Also, make sure that this is done atomically without a DMA
   * interrupt squeezing in there. */
  if (!((DMA2_S7CR & DMA_SxCR_EN) || (DMA2_HISR & DMA_HISR_TCIF7)))
    dma_schedule();

  return n;
}

