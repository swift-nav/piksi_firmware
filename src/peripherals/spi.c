/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f4/dma.h>

#include "ch.h"

#include "spi.h"

/* Defined in usart_rx.c */
extern const u8 dma_irq_lookup[2][8];

/** \addtogroup peripherals
 * \{ */

/** \defgroup spi SPI
 * Functions to setup and use STM32F4 SPI peripherals to communicate with the
 * SwiftNAP FPGA, MAX2769 front-end and the M25 configuration flash.
 * \{ */

static Mutex spi_mutex;
static BinarySemaphore spi_dma_sem;

/** Set up the SPI buses.
 * Set up the SPI peripheral, SPI clocks, SPI pins, and SPI pins' clocks.
 */
void spi_setup(void)
{
  /* Enable SPI1 periperal clock */
  RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
  /* Enable SPI2 periperal clock */
  RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
  /* Enable GPIO clocks for CS lines */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPBEN;

  /* Setup CS line GPIOs */

  /* Deselect FPGA CS */
  gpio_set(GPIOA, GPIO4);
  /* Deselect configuration flash and front-end CS */
  gpio_set(GPIOB, GPIO11 | GPIO12);

  /* FPGA CS */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO4);
  /* Configuration flash CS */
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO12);
  /* Front-end CS */
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);

  /* Setup SPI alternate function */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
  gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 |
                  GPIO15);
  gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);

  /* Setup SPI parameters. */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, 0, 0,
                  SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_ss_output(SPI1); /* Required, see 25.3.1 section about NSS */
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2, 0, 0,
                  SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_ss_output(SPI2); /* Required, see 25.3.1 section about NSS */

  /* Finally enable the SPI. */
  spi_enable(SPI1);
  spi_enable(SPI2);

  chMtxInit(&spi_mutex);
}

/** Deactivate SPI buses.
 * Disable SPI peripherals, SPI clocks, High-Z SPI pins, and disable SPI pins'
 * clocks.
 */
void spi_deactivate(void)
{
  /* Wait until transfers are done per RM0090 page 811 */
  while (SPI1_SR & SPI_SR_BSY) ;
  spi_disable(SPI1);

  /* Disable SPI1 periperal clock */
  RCC_APB2ENR &= ~RCC_APB2ENR_SPI1EN;

  while (SPI2_SR & SPI_SR_BSY) ;
  spi_disable(SPI2);

  /* Disble SPI2 periperal clock */
  RCC_APB1ENR &= ~RCC_APB1ENR_SPI2EN;

  /* Set all SPI GPIOs to inputs with no pull up/down resistors */
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4);
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5 | GPIO6 |
                  GPIO7);
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                  GPIO13 | GPIO14 | GPIO15);

  /* Disble GPIO clocks for CS lines */
  RCC_AHB1ENR &= ~(RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPBEN);
}

/** Drive SPI nCS line low for selected peripheral.
 * \param slave Peripheral to drive chip select for.
 */
void spi_slave_select(u8 slave)
{
  chMtxLock(&spi_mutex);

  switch (slave) {
  case SPI_SLAVE_FPGA:
    gpio_clear(GPIOA, GPIO4);
    break;

  case SPI_SLAVE_FLASH:
    gpio_clear(GPIOB, GPIO12);
    break;

  case SPI_SLAVE_FRONTEND:
    gpio_clear(GPIOB, GPIO11);
    break;
  }
}

/** Drive all SPI nCS lines high.
 * Should be called after an SPI transfer is finished.
 */
void spi_slave_deselect(void)
{
  /* Deselect FPGA CS */
  gpio_set(GPIOA, GPIO4);
  /* Deselect configuration flash and front-end CS */
  gpio_set(GPIOB, GPIO11 | GPIO12);

  chMtxUnlock();
}

static void spi_dma_setup_rx(uint32_t spi, uint32_t dma, u8 stream, u8 channel)
{
  spi_enable_rx_dma(spi);

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
    DMA_SxCR_DIR_PERIPHERAL_TO_MEM |
    /* Enable DMA transfer complete interrupt */
    DMA_SxCR_TCIE |
    /* Increment the memory address after each transfer. */
    DMA_SxCR_MINC |
    /* 8 bit transfers from SPI peripheral. */
    DMA_SxCR_PSIZE_8BIT |
    /* and to memory. */
    DMA_SxCR_MSIZE_8BIT |
    /* Low priority. */
    DMA_SxCR_PL_VERY_HIGH |
    /* The channel selects which request line will trigger a transfer.
     * (see CD00225773.pdf Table 23). */
    DMA_SxCR_CHSEL(channel);

  /* Transfer up to the length of the buffer. */
  DMA_SNDTR(dma, stream) = 0;

  /* DMA from the SPI data register... */
  DMA_SPAR(dma, stream) = &SPI_DR(spi);

  /* Enable DMA interrupts for this stream with the NVIC. */
  if (dma == DMA1)
    nvicEnableVector(dma_irq_lookup[0][stream],
        CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+2));
  else if (dma == DMA2)
    nvicEnableVector(dma_irq_lookup[1][stream],
        CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+2));
}

static void spi_dma_setup_tx(uint32_t spi, uint32_t dma, u8 stream, u8 channel)
{
  spi_enable_tx_dma(spi);

  /* Make sure stream is disabled to start. */
  DMA_SCR(dma, stream) &= ~DMA_SxCR_EN;

  /* Configure the DMA controller. */
  DMA_SCR(dma, stream) = 0;
  DMA_SCR(dma, stream) =
    /* Error interrupts. */
    DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
    DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
    /* Increment the memory address after each transfer. */
    DMA_SxCR_MINC |
    /* 8 bit transfers from SPI peripheral. */
    DMA_SxCR_PSIZE_8BIT |
    /* and to memory. */
    DMA_SxCR_MSIZE_8BIT |
    /* Low priority. */
    DMA_SxCR_PL_VERY_HIGH |
    /* The channel selects which request line will trigger a transfer.
     * (see CD00225773.pdf Table 23). */
    DMA_SxCR_CHSEL(channel);

  /* Transfer up to the length of the buffer. */
  DMA_SNDTR(dma, stream) = 0;

  /* DMA from the SPI data register... */
  DMA_SPAR(dma, stream) = &SPI_DR(spi);

  /* Enable DMA interrupts for this stream with the NVIC. */
  if (dma == DMA1)
    nvicEnableVector(dma_irq_lookup[0][stream],
        CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+2));
  else if (dma == DMA2)
    nvicEnableVector(dma_irq_lookup[1][stream],
        CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+2));
}

void spi1_dma_setup(void)
{
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  spi_dma_setup_rx(SPI1, DMA2, 0, 3);
  spi_dma_setup_tx(SPI1, DMA2, 3, 3);
  chBSemInit(&spi_dma_sem, TRUE);
}

void spi1_xfer_dma(u16 n_bytes, u8 data_in[], const u8 data_out[])
{
  /* We use a static buffer here for DMA transfers as data_in/data_out
   * often are on the stack in CCM which is not accessible by DMA.
   */
  static volatile u8 spi_dma_buf[128];

  memcpy((u8*)spi_dma_buf, data_out, n_bytes);

  /* Setup transmit stream */
  DMA_SM0AR(DMA2, 3) = spi_dma_buf;
  DMA_SNDTR(DMA2, 3) = n_bytes;

  /* Setup receive stream */
  DMA_SM0AR(DMA2, 0) = spi_dma_buf;
  DMA_SNDTR(DMA2, 0) = n_bytes;

  /* We need a memory buffer here to avoid a transfer error */
  asm volatile ("dmb");

  /* Enable the DMA RX channel. */
  DMA_SCR(DMA2, 0) |= DMA_SxCR_EN;
  /* Enable the transmit channel to begin the transaction */
  DMA_SCR(DMA2, 3) |= DMA_SxCR_EN;

  /* Yeild the CPU while we wait for the transaction to complete */
  chBSemWait(&spi_dma_sem);

  if (data_in != NULL)
    memcpy(data_in, (u8*)spi_dma_buf, n_bytes);
}

/** DMA 2 Stream 0 Interrupt Service Routine. (SPI1_RX) */
void dma2_stream0_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  if (dma_get_interrupt_flag(DMA2, 0, DMA_TEIF | DMA_DMEIF))
    screaming_death("DMA SPI1_RX error interrupt");

  /* Disable both receive and transmit streams */
  dma_clear_interrupt_flags(DMA2, 3, DMA_TCIF | DMA_HTIF);
  dma_clear_interrupt_flags(DMA2, 0, DMA_TCIF | DMA_HTIF);

  /* Signal the semaphore to wake up blocking spi1_xfer_dma */
  chBSemSignalI(&spi_dma_sem);

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

/** DMA 2 Stream 3 Interrupt Service Routine. (SPI1_TX) */
void dma2_stream3_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  if (dma_get_interrupt_flag(DMA2, 3, DMA_TEIF | DMA_DMEIF))
    screaming_death("DMA SPI1_TX error interrupt");

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

/** \} */

/** \} */

