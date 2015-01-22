/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/usart.h>

#include "../error.h"
#include "usart.h"

/** \addtogroup peripherals
 * \{ */

/** \addtogroup usart
 * \{ */

const u8 dma_irq_lookup[2][8] = {
  { 11, 12, 13, 14, 15, 16, 17, 47 }, /* DMA1 Stream 0..7. */
  { 56, 57, 58, 59, 60, 68, 69, 70 }, /* DMA2 Stream 0..7. */
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
  chBSemInit(&s->ready_sem, TRUE);

  s->byte_counter = 0;
  s->last_byte_ticks = chTimeNow();

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
  while (DMA_SCR(dma, stream) & DMA_SxCR_EN) ;

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
    nvicEnableVector(dma_irq_lookup[0][stream],
        CORTEX_PRIORITY_MASK(USART_DMA_ISR_PRIORITY));
  else if (dma == DMA2)
    nvicEnableVector(dma_irq_lookup[1][stream],
        CORTEX_PRIORITY_MASK(USART_DMA_ISR_PRIORITY));

  /* These reads clear error flags before enabling DMA */
  (void)USART_SR(usart);
  (void)USART_DR(usart);

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
    nvicDisableVector(dma_irq_lookup[0][s->stream]);
  else if (s->dma == DMA2)
    nvicDisableVector(dma_irq_lookup[1][s->stream]);

  /* Disable DMA stream. */
  DMA_SCR(s->dma, s->stream) &= ~DMA_SxCR_EN;
  while (DMA_SCR(s->dma, s->stream) & DMA_SxCR_EN) ;

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
    screaming_death("USART RX DMA error interrupt");
  }

  if (dma_get_interrupt_flag(s->dma, s->stream, DMA_HTIF | DMA_TCIF)) {
    /* Interrupt is Transmit Complete. We are in circular buffer mode so this
     * probably means we just wrapped the buffer. */

    /* Clear the DMA transmit complete and half complete interrupt flags. */
    dma_clear_interrupt_flags(s->dma, s->stream, DMA_HTIF | DMA_TCIF);

    /* Increment our write wrap counter. */
    s->wr_wraps++;

    chBSemSignalI(&s->ready_sem);
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
  s32 n_read = s->rd_wraps * USART_RX_BUFFER_LEN + s->rd;
  s32 n_written = (s->wr_wraps + 1) * USART_RX_BUFFER_LEN - \
                  DMA_SNDTR(s->dma, s->stream);
  s32 n_available = n_written - n_read;

  if (n_available < 0)
    /* This strange and rare case occurs when NDTR has rolled over but the flag
     * hasn't been raised yet and thus n_wraps hasn't been incremented in the
     * ISR. Simply return 0 this time and the next time this function is called
     * (or at some point) the interrupt will have been triggered and the number
     * of bytes available in the buffer will be a sane amount. */
    n_available = 0;
  else if (n_available > USART_RX_BUFFER_LEN) {
    /* If greater than a whole buffer then we have had an overflow. */
    printf("ERROR: DMA RX buffer overrun\n");
    n_available = 0;
    s->errors++;
    /* Disable and re-enable the DMA channel to get back to a known good
     * state */
    usart_rx_dma_disable(s);
    usart_rx_dma_setup(s, s->usart, s->dma, s->stream, s->channel);
  }

  return n_available;
}

/** Read bytes from the USART RX DMA buffer.
 *
 * \param s The USART DMA state structure.
 * \param data Pointer to a buffer where the received data will be stored.
 * \param len The number of bytes to attempt to read.
 * \param timeout Return if this time passes with no reception.
 * \return The number of bytes successfully read from the DMA receive buffer.
 */
u32 usart_read_dma_timeout(usart_rx_dma_state* s, u8 data[], u32 len, u32 timeout)
{
  u32 n_available;
  do {
    n_available = usart_n_read_dma(s);
  } while ((n_available < len) &&
           (chBSemWaitTimeout(&s->ready_sem, timeout) == RDY_OK));
  n_available = usart_n_read_dma(s);
  u16 n = (len > n_available) ? n_available : len;

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

  s->byte_counter += n;

  return n;
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
  return usart_read_dma_timeout(s, data, len, 0);
}

/**
 * Returns the total bytes divided by the total elapsed seconds since the
 * previous call of this function.
 *
 * \param s The USART DMA state structure
 */
float usart_rx_throughput(usart_rx_dma_state* s)
{
  systime_t now_ticks = chTimeNow();
  float elapsed = ((float)((now_ticks - s->last_byte_ticks) /
    (double)CH_FREQUENCY*1000.0));
  float kbps = s->byte_counter / elapsed;

  s->byte_counter = 0;
  s->last_byte_ticks = now_ticks;

  return kbps;
}

/** \} */

/** \} */

