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

/** Setup the USART for transmission with DMA.
 * This function sets up the DMA controller and additional USART parameters for
 * DMA transmit. The USART must already be configured for normal operation.
 *
 * \param s The USART DMA state structure.
 * \oaram usart The USART base address.
 * \param dma The DMA controller base address.
 * \param stream The DMA stream number to use.
 * \param channel The DMA channel to use. The stream and channel must
 *                correspond to a USART RX channel.
 */
void usart_tx_dma_setup(usart_tx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel)
{
  s->dma = dma;
  s->usart = usart;
  s->stream = stream;
  s->channel = channel;

  s->byte_counter = 0;
  s->last_byte_ticks = chTimeNow();
  
  /* Enable clock to DMA peripheral. */
  if (dma == DMA1)
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  else if (dma == DMA2)
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Enable TX DMA on the USART. */
  usart_enable_tx_dma(usart);

  /* Make sure stream is disabled to start. */
  DMA_SCR(dma, stream) &= ~DMA_SxCR_EN;

  /* Configure the DMA controller. */
  DMA_SCR(dma, stream) = 0;
  DMA_SCR(dma, stream) =
    /* Error interrupts. */
    DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
    /* Transfer complete interrupt. */
    DMA_SxCR_TCIE |
    DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
    /* Increment the memory address after each transfer. */
    DMA_SxCR_MINC |
    /* 4 bytes written to the FIFO from memory at a time */
    DMA_SxCR_MBURST_INCR4 |
    /* 8 bit transfers from USART peripheral. */
    DMA_SxCR_PSIZE_8BIT |
    /* and to memory. */
    DMA_SxCR_MSIZE_8BIT |
    /* TODO: what priority level is necessary? */
    /* Very high priority. */
    DMA_SxCR_PL_VERY_HIGH |
    /* The channel selects which request line will trigger a transfer.
     * (see CD00225773.pdf Table 23). */
    DMA_SxCR_CHSEL(channel);

  /* For now, don't transfer any number of datas
   * (will be set in the initiating function). */
  DMA_SNDTR(dma, stream) = 0;

  /* DMA into the USART data register... */
  DMA_SPAR(dma, stream) = &USART_DR(usart);
  /* ...from the TX buffer. */
  DMA_SM0AR(dma, stream) = s->buff;

  /* TODO: Investigate more about the best FIFO settings. */
  DMA_SFCR(dma, stream) =
    DMA_SxFCR_DMDIS |         /* Enable DMA stream FIFO. */
    DMA_SxFCR_FTH_2_4_FULL |  /* Trigger level 2/4 full. */
    DMA_SxFCR_FEIE;           /* Enable FIFO error interrupt. */

  s->wr = s->rd = 0;  /* Buffer is empty to begin with. */

  /* Enable DMA interrupts for this stream with the NVIC. */
  if (dma == DMA1)
    nvicEnableVector(dma_irq_lookup[0][stream],
        CORTEX_PRIORITY_MASK(USART_DMA_ISR_PRIORITY));
  else if (dma == DMA2)
    nvicEnableVector(dma_irq_lookup[1][stream],
        CORTEX_PRIORITY_MASK(USART_DMA_ISR_PRIORITY));
}

/** Disable USART TX DMA.
 * \param s The USART DMA state structure.
 */
void usart_tx_dma_disable(usart_tx_dma_state* s)
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
  usart_disable_tx_dma(s->usart);
}

/** Calculate the space left in the USART DMA TX buffer.
 * \param s The USART DMA state structure.
 * \return The number of bytes that may be safely written to the buffer.
 */
u32 usart_tx_n_free(usart_tx_dma_state* s)
{
  /* The calculation for the number of bytes in the buffer depends on whether
   * or not the write pointer has wrapped around the end of the buffer. */
  if (s->wr >= s->rd)
    return USART_TX_BUFFER_LEN - 1 - (s->wr - s->rd);
  else
    return (s->rd - s->wr) - 1;
}

/** Helper function that schedules a new transfer with the DMA controller if
 * needed.
 * \param s The USART DMA state structure.
 * */
static void dma_schedule(usart_tx_dma_state* s)
{
  /* TODO: We shouldn't have to check for this now that we are called
   * atomically but leaving it in for now just in case. */
  if (DMA_SCR(s->dma, s->stream) & DMA_SxCR_EN)
    screaming_death("DMA TX scheduled while DMA channel running");

  DMA_SM0AR(s->dma, s->stream) = &(s->buff[s->rd]);

  /* Save the transfer length so we can increment the read index after the
   * transfer is finished. */
  if (s->rd < s->wr)
    /* DMA up until write pointer. */
    s->xfer_len = s->wr - s->rd;
  else
    /* DMA up until the end of the buffer. */
    s->xfer_len = USART_TX_BUFFER_LEN - s->rd;

  /* Set the number of datas in the DMA controller. */
  DMA_SNDTR(s->dma, s->stream) = s->xfer_len;

  /* Clear USART_TC flag */
  USART_SR(s->usart) &= ~USART_SR_TC;

  /* Enable DMA stream to start transfer. */
  DMA_SCR(s->dma, s->stream) |= DMA_SxCR_EN;
}

/** USART TX DMA interrupt service routine.
 * Should be called from the relevant DMA stream ISR.
 * \param s The USART DMA state structure.
 */
void usart_tx_dma_isr(usart_tx_dma_state* s)
{
  if (dma_get_interrupt_flag(s->dma, s->stream,
                             DMA_TEIF | DMA_DMEIF))
    /* TODO: Handle error interrupts! */
    screaming_death("DMA TX error interrupt");

  if (dma_get_interrupt_flag(s->dma, s->stream, DMA_TCIF)) {
    /* Interrupt is Transmit Complete. */

    /* Clear the DMA transmit complete and half complete interrupt flags. */
    dma_clear_interrupt_flags(s->dma, s->stream, DMA_HTIF | DMA_TCIF);

    /* Now that the transfer has finished we can increment the read index. */
    s->rd = (s->rd + s->xfer_len) % USART_TX_BUFFER_LEN;

    if (s->wr != s->rd)
      /* Buffer not empty. */
      dma_schedule(s);
  }

  if (dma_get_interrupt_flag(s->dma, s->stream, DMA_FEIF))
    /* Clear FIFO error flag */
    dma_clear_interrupt_flags(s->dma, s->stream, DMA_HTIF | DMA_FEIF);
}

/** Write out data over the USART using DMA.
 * Note that this function is not reentrant and does not guard against DMA IRQs
 * running at the same time which will also cause spurious behaviours. Ensure
 * that the calling function prevents this from happening.
 *
 * \param s The USART DMA state structure.
 * \param data A pointer to the data to write out.
 * \param len  The number of bytes to write.
 * \return The number of bytes that will be written, may be less than len.
 */
u32 usart_write_dma(usart_tx_dma_state* s, const u8 data[], u32 len)
{
  /* If there is no data to write, just return. */
  if (len == 0) return 0;

  /* Check if the write would cause a buffer overflow, if so only write up to
   * the end of the buffer. */
  u32 n_free = usart_tx_n_free(s);
  if (len > n_free)
    return 0;

  u32 old_wr = s->wr;
  s->wr = (s->wr + len) % USART_TX_BUFFER_LEN;

  if (old_wr + len <= USART_TX_BUFFER_LEN)
    memcpy(&(s->buff[old_wr]), data, len);
  else {
    /* Deal with case where write wraps the buffer. */
    memcpy(&(s->buff[old_wr]), &data[0], USART_TX_BUFFER_LEN - old_wr);
    memcpy(&(s->buff[0]), &data[USART_TX_BUFFER_LEN - old_wr],
           len - (USART_TX_BUFFER_LEN - old_wr));
  }

  /* Check if there is a DMA transfer either in progress or waiting for its
   * interrupt to be serviced. Its very important to also check the interrupt
   * flag as EN will be cleared when the transfer finishes but we really need
   * to make sure the ISR has been run to finish up the bookkeeping for the
   * transfer. Also, make sure that this is done atomically without a DMA
   * interrupt squeezing in there. */
  if (!((DMA_SCR(s->dma, s->stream) & DMA_SxCR_EN) ||
        dma_get_interrupt_flag(s->dma, s->stream, DMA_TCIF)))
    dma_schedule(s);

  s->byte_counter += len;

  return len;
}

/**
 * Returns the total bytes divided by the total elapsed seconds since the
 * previous call of this function.
 *
 * \param s The USART DMA state structure
 */
float usart_tx_throughput(usart_tx_dma_state* s)
{
  systime_t now_ticks = chTimeNow();
  float elapsed = ((float)((now_ticks - s->last_byte_ticks) /
    (double)CH_FREQUENCY));
  float kbps = s->byte_counter / (elapsed * 1000.0);

  s->byte_counter = 0;
  s->last_byte_ticks = now_ticks;
  
  return kbps;
}

/** \} */

/** \} */

