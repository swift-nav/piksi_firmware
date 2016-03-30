/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>

#include <libswiftnav/logging.h>

#include "peripherals/usart.h"
#include "error.h"

#define USART_TX_BUFFER_LEN 2048
#define USART_RX_BUFFER_LEN 2048

#define USART1_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_SERIAL_USART1_TX_DMA_STREAM,                   \
                       STM32_USART1_TX_DMA_CHN)

#define USART3_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_SERIAL_USART3_TX_DMA_STREAM,                   \
                       STM32_USART3_TX_DMA_CHN)

#define USART6_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_SERIAL_USART6_TX_DMA_STREAM,                   \
                       STM32_USART6_TX_DMA_CHN)

struct usart_support_s {
  USART_TypeDef *usart;
  u32 dmamode;
  struct usart_rx_dma_state {
    /** USART RX DMA buffer. DMA xfers from USART to buffer, message processing
     * routine reads out of buffer. */
    u8 buff[USART_RX_BUFFER_LEN];
    u32 rd;       /**< Address of next byte to read out of buffer.  */
    /* TODO : is u32 big enough for rd_wraps and wr_wraps? */
    u32 rd_wraps; /**< Number of times rd has wrapped around the buffer. */
    u32 wr_wraps; /**< Number of times wr has wrapped around the buffer. */
    const stm32_dma_stream_t *dma;     /**< DMA for particular USART. */
    binary_semaphore_t ready;
  } rx;
  struct usart_tx_dma_state {
    /** USART TX DMA buffer. DMA xfers from buffer to USART_DR. */
    u8 buff[USART_TX_BUFFER_LEN];
    u32 rd;       /**< Address of next byte to read out of buffer. */
    u32 wr;       /**< Next buffer address to write to. */
    u32 xfer_len; /**< Number of bytes to DMA from buffer to USART_DR. */
    const stm32_dma_stream_t *dma;     /**< DMA for particular USART. */
    bool busy;
  } tx;
} SD1, SD3, SD6;

static void usart_rx_dma_isr(struct usart_rx_dma_state* s, u32 flags);
static void usart_tx_dma_isr(struct usart_tx_dma_state* s, u32 flags);

static void usart_support_init_tx(struct usart_support_s *sd)
{
  /* Setup TX DMA */
  dmaStreamSetMode(sd->tx.dma, sd->dmamode | STM32_DMA_CR_DIR_M2P |
                               STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
  dmaStreamSetPeripheral(sd->tx.dma, &sd->usart->DR);
}

static void usart_support_init_rx(struct usart_support_s *sd)
{
  /* Setup RX DMA */
  dmaStreamSetMode(sd->rx.dma, sd->dmamode | STM32_DMA_CR_DIR_P2M |
                               STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE |
                               STM32_DMA_CR_CIRC);
  dmaStreamSetTransactionSize(sd->rx.dma, USART_RX_BUFFER_LEN);
  dmaStreamSetPeripheral(sd->rx.dma, &sd->usart->DR);
  dmaStreamSetMemory0(sd->rx.dma, sd->rx.buff);
  chBSemObjectInit(&sd->rx.ready, TRUE);
  dmaStreamEnable(sd->rx.dma);

}

void usart_support_init(void)
{
  SD1.usart = USART1;
  SD1.tx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART1_TX_DMA_STREAM);
  dmaStreamAllocate(SD1.tx.dma, STM32_SERIAL_USART1_PRIORITY,
                    (stm32_dmaisr_t)usart_tx_dma_isr, &SD1.tx);
  SD1.rx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART1_RX_DMA_STREAM);
  dmaStreamAllocate(SD1.rx.dma, STM32_SERIAL_USART1_PRIORITY,
                    (stm32_dmaisr_t)usart_rx_dma_isr, &SD1.rx);
  SD1.dmamode = STM32_DMA_CR_CHSEL(USART1_TX_DMA_CHANNEL) |
                STM32_DMA_CR_PL(STM32_SERIAL_USART1_DMA_PRIORITY);
  usart_support_init_tx(&SD1);
  usart_support_init_rx(&SD1);
  rccEnableUSART1(FALSE);

  SD3.usart = USART3;
  SD3.tx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART3_TX_DMA_STREAM);
  dmaStreamAllocate(SD3.tx.dma, STM32_SERIAL_USART3_PRIORITY,
                    (stm32_dmaisr_t)usart_tx_dma_isr, &SD3.tx);
  SD3.rx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART3_RX_DMA_STREAM);
  dmaStreamAllocate(SD3.rx.dma, STM32_SERIAL_USART3_PRIORITY,
                    (stm32_dmaisr_t)usart_rx_dma_isr, &SD3.rx);
  SD3.dmamode = STM32_DMA_CR_CHSEL(USART3_TX_DMA_CHANNEL) |
                STM32_DMA_CR_PL(STM32_SERIAL_USART3_DMA_PRIORITY);
  usart_support_init_tx(&SD3);
  usart_support_init_rx(&SD3);
  rccEnableUSART3(FALSE);

  SD6.usart = USART6;
  SD6.tx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART6_TX_DMA_STREAM);
  dmaStreamAllocate(SD6.tx.dma, STM32_SERIAL_USART6_PRIORITY,
                    (stm32_dmaisr_t)usart_tx_dma_isr, &SD6.tx);
  SD6.rx.dma = STM32_DMA_STREAM(STM32_SERIAL_USART6_RX_DMA_STREAM);
  dmaStreamAllocate(SD6.rx.dma, STM32_SERIAL_USART6_PRIORITY,
                    (stm32_dmaisr_t)usart_rx_dma_isr, &SD6.rx);
  SD6.dmamode = STM32_DMA_CR_CHSEL(USART6_TX_DMA_CHANNEL) |
                STM32_DMA_CR_PL(STM32_SERIAL_USART6_DMA_PRIORITY);
  usart_support_init_tx(&SD6);
  usart_support_init_rx(&SD6);
  rccEnableUSART6(FALSE);
}

void usart_support_set_parameters(void *sd, u32 baud)
{
  if (sd == NULL)
    return;

  USART_TypeDef *u = ((struct usart_support_s *)sd)->usart;

  if ((u == USART1) || (u == USART6))
    u->BRR = STM32_PCLK2 / baud;
  else
    u->BRR = STM32_PCLK1 / baud;

  u->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
  u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

  u->SR = 0;
  (void)u->SR;  /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/
}

void usart_support_disable(void *sd)
{
  if (sd == NULL)
    return;

  USART_TypeDef *u = ((struct usart_support_s *)sd)->usart;
  u->CR1 = 0;
  u->CR2 = 0;
  u->CR3 = 0;
}

static void usart_rx_dma_isr(struct usart_rx_dma_state* s, u32 flags)
{
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0)
    screaming_death("USART RX DMA error interrupt");

  chSysLockFromISR();
  /* Increment our write wrap counter. */
  s->wr_wraps++;
  chBSemSignalI(&s->ready);
  chSysUnlockFromISR();
}

u32 usart_support_n_read(void *sd)
{
  struct usart_support_s *u = (struct usart_support_s *)sd;
  struct usart_rx_dma_state *s = &u->rx;

  s32 n_read = s->rd_wraps * USART_RX_BUFFER_LEN + s->rd;
  s32 n_written = (s->wr_wraps + 1) * USART_RX_BUFFER_LEN -
                  dmaStreamGetTransactionSize(s->dma);
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
    log_error("DMA RX buffer overrun");
    n_available = 0;
    /* Disable and re-enable the DMA channel to get back to a known good
     * state */
    dmaStreamDisable(s->dma);
    usart_support_init_rx(u);
  }

  return n_available;
}

u32 usart_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout)
{
  struct usart_rx_dma_state *s = &((struct usart_support_s *)sd)->rx;

  u32 n_available;
  do {
    n_available = usart_support_n_read(sd);
  } while ((n_available < len) &&
           (chBSemWaitTimeout(&s->ready, timeout) == MSG_OK));
  n_available = usart_support_n_read(sd);
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

  return n;
}

u32 usart_support_tx_n_free(void *sd)
{
  struct usart_tx_dma_state *s = &((struct usart_support_s *)sd)->tx;
  /* The calculation for the number of bytes in the buffer depends on whether
   * or not the write pointer has wrapped around the end of the buffer. */
  if (s->wr >= s->rd)
    return USART_TX_BUFFER_LEN - 1 - (s->wr - s->rd);
  else
    return (s->rd - s->wr) - 1;
}

static void dma_schedule(struct usart_tx_dma_state *s)
{
  dmaStreamSetMemory0(s->dma, &s->buff[s->rd]);
  /* Save the transfer length so we can increment the read index after the
   * transfer is finished. */
  if (s->rd < s->wr)
    /* DMA up until write pointer. */
    s->xfer_len = s->wr - s->rd;
  else
    /* DMA up until the end of the buffer. */
    s->xfer_len = USART_TX_BUFFER_LEN - s->rd;

  dmaStreamSetTransactionSize(s->dma, s->xfer_len);
  dmaStreamEnable(s->dma);
  s->busy = true;
}

static void usart_tx_dma_isr(struct usart_tx_dma_state* s, u32 flags)
{
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0)
    screaming_death("USART TX DMA error interrupt");

  osalSysLockFromISR();
  /* Now that the transfer has finished we can increment the read index. */
  s->rd = (s->rd + s->xfer_len) % USART_TX_BUFFER_LEN;

  if (s->wr != s->rd)
    /* Buffer not empty. */
    dma_schedule(s);
  else
    s->busy = false;
  osalSysUnlockFromISR();
}

u32 usart_support_write(void *sd, const u8 data[], u32 len)
{
  struct usart_tx_dma_state *s = &((struct usart_support_s *)sd)->tx;

  /* If there is no data to write, just return. */
  if (len == 0) return 0;

  chSysLock();

  /* Check if the write would cause a buffer overflow, if so only write up to
   * the end of the buffer. */
  u32 n_free = usart_tx_n_free(sd);
  if (len > n_free) {
    chSysUnlock();
    return 0;
  }

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
  if (!s->busy)
    dma_schedule(s);

  chSysUnlock();
  return len;
}

