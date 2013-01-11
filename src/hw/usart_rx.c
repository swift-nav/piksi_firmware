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
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/dma.h>

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
  /* Set up the USART6 RX DMA and interrupts. */

  /* Enable clock to DMA peripheral. */
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Enable RX DMA on the USART. */
  usart_enable_rx_dma(USART6);

  /* USART1 RX - DMA2, stream 5, channel 4. */
  /* USART6 RX - DMA2, stream 2, channel 5. */

  /* Make sure stream is disabled to start. */
  DMA2_S2CR &= ~DMA_SxCR_EN;

  /* Supposed to wait until enable bit reads '0' before we write to registers */
  while (DMA2_S2CR & DMA_SxCR_EN) {
    __asm__("nop");
  }

  /* Configure the DMA controller. */
  DMA2_S2CR = 0;
              /* Error interrupts. */
  DMA2_S2CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
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
               * transfer. In this case, channel 5 = UART6_RX
               * (see CD00225773.pdf Table 23). */
              DMA_SxCR_CHSEL(5);

  /* Transfer up the the length of the buffer. */
  DMA2_S2NDTR = USART_RX_BUFFER_LEN;

  DMA2_S2PAR = &USART6_DR; /* DMA from the USART6 data register. */
  DMA2_S2M0AR = rx_buff;      /* to the rx_buff. */

  // TODO: Investigate more about the best FIFO settings.
//  DMA2_S2FCR = DMA_SxFCR_DMDIS |         /* Enable DMA stream FIFO. */
//               DMA_SxFCR_FTH_2_4_FULL |  /* Trigger level 1/2 full. */
//               DMA_SxFCR_FEIE;           /* Enable FIFO error interrupt. */

  rx_rd = 0;  /* Buffer is empty to begin with. */
  rd_wraps = wr_wraps = 0;

  /* Enable DMA2 Stream 2 (RX) interrupts with the NVIC. */
  nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);

  /* Enable the DMA channel. */
  DMA2_S2CR |= DMA_SxCR_EN;
}

void dma2_stream2_isr()
{
  //make sure TCIF2 and HTIF2 bits are the only ones that are high
  if ((DMA2_LISR & (DMA_LISR_TCIF2 | DMA_LISR_HTIF2)) && !(DMA2_LISR & ~(DMA_LISR_TCIF2 | DMA_LISR_HTIF2))) {
    /* Interrupt is Transmit Complete. We are in circular buffer mode
     * so this probably means we just wrapped the buffer.
     */

    /* Clear the DMA transmit complete and half complete interrupt flags. */
    DMA2_LIFCR = (DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);

    /* Increment our write wrap counter. */
    wr_wraps++;
  } else {
    // TODO: Handle error interrupts! */
    speaking_death("DMA RX error interrupt");
  }
}

u32 usart_n_read_dma()
{
  s32 n_read = rd_wraps*USART_RX_BUFFER_LEN + rx_rd;
  s32 n_written = (wr_wraps+1)*USART_RX_BUFFER_LEN - DMA2_S2NDTR;
  s32 n_available = n_written - n_read;
  /* If this case occurs we have had strange timing between the 
   * call to this function, the rollover of NDTR, and the interrupt
   * flag for the rollover of NDTR being raised, ie NDTR has rolled
   * over but the flag hasn't been raised and thus n_wraps hasn't
   * been incremented in the ISR yet. There shouldn't be
   * any case in which more bytes are actually read out of the buffer
   * than have been written in - usart_read_dma takes care of this 
   * We will simply return 0 this call and the next time this
   * function is called (or at some point) the interrupt will have
   * been resolved and the number of bytes available in the buffer
   * will be a sane amount */
  if (n_available < 0) {
    n_available = 0;
  // If greater than a whole buffer then we have had an overflow
  } else if (n_available > USART_RX_BUFFER_LEN) {
    speaking_death("DMA RX buffer overrun");
  }
  return n_available;
}

u32 usart_read_dma(u8 data[], u32 len)
{
  u16 n_to_read = usart_n_read_dma();
  u16 n = (len > n_to_read) ? n_to_read : len;

  if (rx_rd + n < USART_RX_BUFFER_LEN) {
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

