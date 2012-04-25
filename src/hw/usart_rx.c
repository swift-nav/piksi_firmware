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

static u8 rx_buff[USART_RX_BUFFER_LEN];
static u32 rx_rd;
static u32 rd_wraps;
static u32 wr_wraps;

/** Setup the USART for recieve with DMA. This function sets up the DMA
 * controller and additional USART parameters for DMA receive. The USART must
 * first be configured using \ref usart_setup_common. */
void usart_rx_dma_setup(void) {
  /* Set up the USART1 RX DMA and interrupts. */

  /* Enable clock to DMA peripheral. */
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Enable RX DMA on the USART. */
  usart_enable_rx_dma(USART1);

  /* USART1 RX - DMA2, stream 5, channel 4. */

  /* Make sure stream is disabled to start. */
  DMA2_S5CR &= ~DMA_SxCR_EN;
  DMA2_S5CR = 0;

  /* Configure the DMA controller. */
              /* Error interrupts. */
  DMA2_S5CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
              /* Transfer complete interrupt. */
              DMA_SxCR_TCIE |
              /* Enable circular buffer mode. */
              DMA_SxCR_CIRC |
              DMA_SxCR_DIR_PERIPHERAL_TO_MEM |
              /* Increment the memory address after each transfer. */
              DMA_SxCR_MINC |
              /* 8 bit transfers from USART peripheral. */
              DMA_SxCR_PSIZE_8BIT |
              /* and to memory. */
              DMA_SxCR_MSIZE_8BIT |
              /* Low priority. */
              DMA_SxCR_PL_LOW |
              /* The channel selects which request line will trigger a
               * transfer. In this case, channel 4 = UART1_RX
               * (see CD00225773.pdf Table 23). */
              DMA_SxCR_CHSEL(4);

  /* Transfer up the the length of the buffer. */
  DMA2_S5NDTR = USART_RX_BUFFER_LEN;

  DMA2_S5PAR = &USART1_DR; /* DMA from the USART1 data register. */
  DMA2_S5M0AR = rx_buff;      /* to the rx_buff. */

  // TODO: Investigate more about the best FIFO settings.
  DMA2_S5FCR = DMA_SxFCR_DMDIS |         /* Enable DMA stream FIFO. */
               DMA_SxFCR_FTH_2_4_FULL |  /* Trigger level 1/2 full. */
               DMA_SxFCR_FEIE;           /* Enable FIFO error interrupt. */

  rx_rd = 0;  /* Buffer is empty to begin with. */
  rd_wraps = wr_wraps = 0;

  /* Enable DMA2 Stream 5 (RX) interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);

  /* Enable the DMA channel. */
  DMA2_S5CR |= DMA_SxCR_EN;
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
    wr_wraps++;
  } else {
    // TODO: Handle error interrupts! */
    speaking_death("DMA RX error interrupt");
  }
}

u32 usart_n_read_dma()
{
  __asm__("CPSID i;");
  // Compare number of bytes written to the number read, if greater
  // than a whole buffer then we have had an overflow.
  u32 n_read = rd_wraps*USART_RX_BUFFER_LEN + rx_rd;
  u32 n_written = (wr_wraps+1)*USART_RX_BUFFER_LEN - DMA2_S5NDTR;
  __asm__("CPSIE i;");
  if (n_written - n_read > USART_RX_BUFFER_LEN) {
    // My buffer runneth over.
    speaking_death("DMA RX buffer overrun");
  }
  return n_written - n_read;
}

u32 usart_read_dma(u8 data[], u32 len)
{
  u16 n_to_read = usart_n_read_dma();
  u16 n = (len > n_to_read) ? n_to_read : len;

  if (rx_rd + n < USART_RX_BUFFER_LEN) {
    /* TODO: this is worng! dest <- src */
    memcpy(data, &rx_buff[rx_rd], n);
    rx_rd += n;
  } else {
    rd_wraps++;
    memcpy(&data[0], &rx_buff[rx_rd], USART_RX_BUFFER_LEN - rx_rd);
    memcpy(&data[USART_RX_BUFFER_LEN - rx_rd], &rx_buff[0], n - USART_RX_BUFFER_LEN + rx_rd);
    rx_rd = n - USART_RX_BUFFER_LEN + rx_rd;
  }

  return n;
}

