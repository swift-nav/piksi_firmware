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
#include <stdio.h>
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
  DMA2_S2CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_TCIE |
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
  DMA2_S2FCR = DMA_SxFCR_DMDIS |         /* Enable DMA stream FIFO. */
               DMA_SxFCR_FTH_2_4_FULL |  /* Trigger level 1/2 full. */
               DMA_SxFCR_FEIE;           /* Enable FIFO error interrupt. */

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
//  if (DMA2_LISR & DMA_LISR_TCIF2) {
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
  static u32 d1_n_written, d1_n_read;
  static u32 d1_ndtr_old, d1_ndtr_new;
  static u32 d1_wr_wraps_old, d1_wr_wraps_new;
  volatile u32 wr_wraps_old, ndtr_old;
  volatile u32 wr_wraps_new, ndtr_new;
  u32 xtra_wr_wraps = 1;

  u32 n_written;
  u32 n_read;

  // It appears that it is possible for the NDTR register to roll over
  // before "A" without the interrupt being called at any point within the
  // function - this delay (hopefully) gives enough time for the interrupt
  // to be called before we read wr_wraps
  if ((wr_wraps_new == wr_wraps_old) && (d1_wr_wraps_old == d1_wr_wraps_new) && (d1_wr_wraps_new == wr_wraps_new) && ((ndtr_old == USART_RX_BUFFER_LEN) && (ndtr_new == USART_RX_BUFFER_LEN)) && (d1_ndtr_old == 1 && d1_ndtr_new == 1)) {
//    u32  = ;
    for (u32 i; i<10000; i++) {
      __asm__("nop");
    }
//    printf("");
  }

  wr_wraps_old = wr_wraps;
  //A : possible interrupt here that would cause our observations to be funky
  //    ie wr_wraps observation is out of date but ndtr has rolled over
  ndtr_old = DMA2_S2NDTR;
  //B : possible interrupt here
  __asm__("CPSID i;");
  //C : possible rollover of DMA2_S2NDTR here - C's are equivalent as wr_wraps
  //    can't be incremented with interrupts disabled
  wr_wraps_new = wr_wraps;
  //C : possible rollover of DMA2_S2NDTR here - C's are equivalent as wr_wraps
  //    can't be incremented with interrupts disabled
  ndtr_new = DMA2_S2NDTR;
  /* We assume that DMA2_S2NDTR does not roll over at more than one of A, B, C,
   * so we have 4 possible cases :
   * 1 : no rollover between old and new observations
   *     wr_wraps_new == wr_wraps_old, ndtr_old >= ndtr_new
   *     Use new values for n_read / n_written calcs
   * 2 : rollover at A. 
   *     wr_wraps_new == wr_wraps_old + 1, ndtr_old >= ndtr_new
   *     Use new values for n_read / n_written calcs
   * 3 : rollover at B.
   *     wr_wraps_new == wr_wraps_old + 1, ndtr_old < ndtr_new
   *     Use new values for n_read / n_written calcs
   * 4 : rollover at C
   *     wr_wraps_new == wr_wraps_old, ndtr_old < ndtr_new
   *     use wr_wraps_new + 1 and ndtr_new for calcs
   */
  if ((wr_wraps_new == wr_wraps_old) && ((ndtr_old < ndtr_new))) {
    xtra_wr_wraps = 2;
  } else if ((wr_wraps_new == wr_wraps_old) && (d1_wr_wraps_old == d1_wr_wraps_new) && (d1_wr_wraps_new == wr_wraps_new) && ((ndtr_old == USART_RX_BUFFER_LEN) && (ndtr_new == USART_RX_BUFFER_LEN)) && (d1_ndtr_old == 1 && d1_ndtr_new == 1)) {
    xtra_wr_wraps = 2;
  }
  __asm__("CPSIE i;");
  n_written = (wr_wraps_new+xtra_wr_wraps)*USART_RX_BUFFER_LEN - ndtr_new;
  n_read = rd_wraps*USART_RX_BUFFER_LEN + rx_rd;
//  if ((((ndtr_old == USART_RX_BUFFER_LEN) && (ndtr_new == USART_RX_BUFFER_LEN)) || ((ndtr_old == 1) && (ndtr_new == 1))) && (n_written > 0)) {
  if (((ndtr_old == USART_RX_BUFFER_LEN) && (ndtr_new == USART_RX_BUFFER_LEN)) && (n_written > 0)) {
    printf("ndtr_old = %d\n",(int)ndtr_old);
    printf("ndtr_new = %d\n",(int)ndtr_new);
    printf("d1_n_written = %d\n",(int)d1_n_written);
    printf("d1_n_read = %d\n",(int)d1_n_read);
    printf("d1_wr_wraps_old = %d\n",(int)d1_wr_wraps_old);
    printf("d1_wr_wraps_new = %d\n",(int)d1_wr_wraps_new);
    printf("d1_ndtr_old = %d\n",(int)d1_ndtr_old);
    printf("d1_ndtr_new = %d\n",(int)d1_ndtr_new);
    printf("wr_wraps_old = %d\n",(int)wr_wraps_old);
    printf("wr_wraps_new = %d\n",(int)wr_wraps_new);
    printf("xtra_wr_wraps = %d\n",(int)xtra_wr_wraps);
    printf("n_written = %d\n",(int)n_written);
    printf("n_read = %d\n",(int)n_read);
    printf("\n");
//    for(u32 i=0; i<1000; i++)
//      __asm__("nop");
  }
  // Compare number of bytes written to the number read, if greater
  // than a whole buffer then we have had an overflow.
  if (n_written - n_read > USART_RX_BUFFER_LEN) {
     //Wait for any messages that are still transmitting to finish
     for (u32 i; i<10000; i++){
       __asm__("nop");
     }
     printf("last wr_wraps before going into speaking death : %d\n", (int)wr_wraps);
     for (u32 i; i<10000; i++){
       __asm__("nop");
     }
     while(usart_tx_n_free() < USART_TX_BUFFER_LEN){
       __asm__("nop");
     }
     speaking_death("DMA RX buffer overrun");
  }
  d1_n_written = n_written;
  d1_ndtr_old = ndtr_old;
  d1_wr_wraps_old = wr_wraps_old;
  d1_ndtr_new = ndtr_new;
  d1_wr_wraps_new = wr_wraps_new;
  d1_n_read = n_read;
  return n_written - n_read;
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

//u32 usart_read_dma(u8 data[], u32 len)
//{
//  u16 n_avail = usart_n_read_dma();
//  if (len > n_avail)
//    speaking_death("Tried to read more bytes out of RX buffer than were in RX buffer");
//  u16 n = len;
//
//  if (rx_rd + n < USART_RX_BUFFER_LEN) {
//    memcpy(data, &rx_buff[rx_rd], n);
//    rx_rd += n;
//  } else {
//    rd_wraps++;
//    memcpy(&data[0], &rx_buff[rx_rd], USART_RX_BUFFER_LEN - rx_rd);
//    memcpy(&data[USART_RX_BUFFER_LEN - rx_rd], &rx_buff[0], n - USART_RX_BUFFER_LEN + rx_rd);
//    rx_rd = n - USART_RX_BUFFER_LEN + rx_rd;
//  }
//
//  return n;
//}
