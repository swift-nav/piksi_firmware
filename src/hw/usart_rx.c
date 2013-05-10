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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/dma.h>

#include "../error.h"
#include "usart.h"

const u8 dma_irq_lookup[2][8] = {
  {11, 12, 13, 14, 15, 16, 17, 47}, /* DMA1 Stream 0..7. */
  {56, 57, 58, 59, 60, 68, 69, 70}, /* DMA2 Stream 0..7. */
};

/** Setup the USART for receive with DMA.
 * This function sets up the DMA controller and additional USART parameters for
 * DMA receive. The USART must already be configured for normal operation.
 *
 * \param s The USART DMA state structure.
 * \oaram usart The USART base address.
 * \param dma The DMA controller base address.
 * \param stream The DMA stream number to use.
 * \param channel The DMA channel to use. The stream and channel must
 *                correspond to a USART RX channel.
 */
void usart_rx_dma_setup(usart_rx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel)
{
  s->dma = dma;
  s->usart = usart;
  s->stream = stream;
  s->channel = channel;

  /* Enable clock to DMA peripheral. */
  if (dma == DMA1)
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  else if (dma == DMA2)
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Enable RX DMA on the USART. */
  usart_enable_rx_dma(usart);

  /* Make sure stream is disabled to start. */
  DMA_SCR(dma, stream) &= ~DMA_SxCR_EN;

  /* RM0090 - 9.3.17 : Supposed to wait until enable bit reads '0' before we
   * write to registers. */
  while (DMA_SCR(dma, stream) & DMA_SxCR_EN);

  /* RM0090 - 9.3.17 : Supposed to clear any interrupts in DMA status register
   * before we reconfigure registers. */
  dma_clear_interrupt_flags(dma, stream, DMA_ISR_FLAGS);

  /* Configure the DMA controller. */
  DMA_SCR(dma, stream) = 0;
  DMA_SCR(dma, stream) =
    /* Error interrupts. */
    DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
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
    /* The channel selects which request line will trigger a transfer.
     * (see CD00225773.pdf Table 23). */
    DMA_SxCR_CHSEL(channel);

  /* Transfer up to the length of the buffer. */
  DMA_SNDTR(dma, stream) = USART_RX_BUFFER_LEN;

  /* DMA from the USART data register... */
  DMA_SPAR(dma, stream) = &USART_DR(usart);
  /* ...to the RX buffer. */
  DMA_SM0AR(dma, stream) = s->buff;

  /* Buffer is empty to begin with. */
  s->rd = 0;
  s->rd_wraps = s->wr_wraps = 0;

  /* Enable DMA interrupts for this stream with the NVIC. */
  if (dma == DMA1)
    nvic_enable_irq(dma_irq_lookup[0][stream]);
  else if (dma == DMA2)
    nvic_enable_irq(dma_irq_lookup[1][stream]);

  /* Enable the DMA channel. */
  DMA_SCR(dma, stream) |= DMA_SxCR_EN;
}

/** Disable USART RX DMA.
 * \param s The USART DMA state structure.
 */
void usart_rx_dma_disable(usart_rx_dma_state* s)
{
  /* Disable DMA stream interrupts with the NVIC. */
  if (s->dma == DMA1)
    nvic_disable_irq(dma_irq_lookup[0][s->stream]);
  else if (s->dma == DMA2)
    nvic_disable_irq(dma_irq_lookup[1][s->stream]);

  /* Disable DMA stream. */
  DMA_SCR(s->dma, s->stream) &= ~DMA_SxCR_EN;
  while (DMA_SCR(s->dma, s->stream) & DMA_SxCR_EN);

  /* Disable RX DMA on the USART. */
  usart_disable_rx_dma(s->usart);

  /* Clear the DMA transmit complete and half complete interrupt flags. */
  dma_clear_interrupt_flags(s->dma, s->stream, DMA_HTIF | DMA_TCIF);
}

/** USART RX DMA interrupt service routine.
 * Should be called from the relevant DMA stream ISR.
 * \param s The USART DMA state structure.
 */
void usart_rx_dma_isr(usart_rx_dma_state* s)
{
  if (dma_get_interrupt_flag(s->dma, s->stream,
                             DMA_TEIF | DMA_DMEIF | DMA_FEIF)) {
    /* TODO: Handle error interrupts! */
    speaking_death("USART RX DMA error interrupt");
  }

  if (dma_get_interrupt_flag(s->dma, s->stream, DMA_HTIF | DMA_TCIF)) {
    /* Interrupt is Transmit Complete. We are in circular buffer mode so this
     * probably means we just wrapped the buffer. */

    /* Clear the DMA transmit complete and half complete interrupt flags. */
    dma_clear_interrupt_flags(s->dma, s->stream, DMA_HTIF | DMA_TCIF);

    /* Increment our write wrap counter. */
    s->wr_wraps++;
  }

  /* Note: When DMA is re-enabled after bootloader it appears ISR can get
   * called without any of the bits of DMA_LISR being high */
}

/** Returns a lower bound on the number of bytes in the DMA receive buffer.
 * Also checks for buffer overrun conditions.
 * \param s The USART DMA state structure.
 */
u32 usart_n_read_dma(usart_rx_dma_state* s)
{
  s32 n_read = s->rd_wraps*USART_RX_BUFFER_LEN + s->rd;
  s32 n_written = (s->wr_wraps+1)*USART_RX_BUFFER_LEN - \
                  DMA_SNDTR(s->dma, s->stream);
  s32 n_available = n_written - n_read;

  if (n_available < 0) {
    /* This strange and rare case occurs when NDTR has rolled over but the flag
     * hasn't been raised yet and thus n_wraps hasn't been incremented in the
     * ISR. Simply return 0 this time and the next time this function is called
     * (or at some point) the interrupt will have been triggered and the number
     * of bytes available in the buffer will be a sane amount. */
    n_available = 0;
  } else if (n_available > USART_RX_BUFFER_LEN) {
    /* If greater than a whole buffer then we have had an overflow. */
    speaking_death("DMA RX buffer overrun");
  }

  return n_available;
}

/** Read bytes from the USART RX DMA buffer.
 *
 * \param s The USART DMA state structure.
 * \param data Pointer to a buffer where the received data will be stored.
 * \param len The number of bytes to attempt to read.
 * \return The number of bytes successfully read from the DMA receive buffer.
 */
u32 usart_read_dma(usart_rx_dma_state* s, u8 data[], u32 len)
{
  u16 n_to_read = usart_n_read_dma(s);
  u16 n = (len > n_to_read) ? n_to_read : len;

  if (s->rd + n < USART_RX_BUFFER_LEN) {
    memcpy(data, &(s->buff[s->rd]), n);
    s->rd += n;
  } else {
    s->rd_wraps++;
    memcpy(&data[0], &(s->buff[s->rd]), USART_RX_BUFFER_LEN - s->rd);
    memcpy(&data[USART_RX_BUFFER_LEN - s->rd],
           &(s->buff[0]), n - USART_RX_BUFFER_LEN + s->rd);
    s->rd = n - USART_RX_BUFFER_LEN + s->rd;
  }

  return n;
}

