/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "axi_dma.h"

#include "axi_dma_regs.h"
#include "zynq7000.h"
#include "gic.h"
#include "hal.h"

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

static void axi_dma_irq_handler(void *context);

#if (ZYNQ7000_AXI_DMA_USE_AXI_DMA0 == TRUE) && !defined(__DOXYGEN__)
AXIDMADriver AXIDMADriver1;
#endif

#if (ZYNQ7000_AXI_DMA_USE_AXI_DMA1 == TRUE) && !defined(__DOXYGEN__)
AXIDMADriver AXIDMADriver2;
#endif

/** Begin an AXI DMA transfer.
 *
 * \param ddp           Pointer to the axi_dma_dir_driver_t object.
 * \param data          Data to be transferred.
 * \param data_length   Length of the data to be transferred.
 * \param callback      Callback to be executed on completion.
 *
 * \note The callback will be executed from interrupt context.
 */
static void axi_dma_dir_transfer_begin(axi_dma_dir_driver_t *ddp,
                                       const uint8_t *data,
                                       uint32_t data_length,
                                       axi_dma_callback_t callback)
{
  axi_dma_dir_t *axi_dma_dir = (axi_dma_dir_t *)ddp->axi_dma_dir;
  osalDbgAssert(axi_dma_dir != 0, "DMA dir not present");

  ddp->callback = callback;

  axi_dma_dir->ADDR_LSB = (uint32_t)data;
  COMPILER_BARRIER(); /* Make sure LENGTH field is written last */
  axi_dma_dir->LENGTH = data_length;
}

/** Start an AXI DMA direction driver.
 *
 * \param ddp     Pointer to the axi_dma_dir_driver_t object.
 */
static void axi_dma_dir_start(axi_dma_dir_driver_t *ddp)
{
  axi_dma_dir_t *axi_dma_dir = (axi_dma_dir_t *)ddp->axi_dma_dir;
  osalDbgAssert(axi_dma_dir != 0, "DMA dir not present");

  /* Clear COMPLETE and ERROR interrupt status */
  axi_dma_dir->SR = AXI_DMA_INT_COMPLETE_Msk |
                    AXI_DMA_INT_ERROR_Msk;

  /* Enable COMPLETE and ERROR interrupts */
  axi_dma_dir->CR |= AXI_DMA_INT_COMPLETE_Msk |
                     AXI_DMA_INT_ERROR_Msk;

  /* Set ENABLE bit */
  axi_dma_dir->CR |= AXI_DMA_CR_RUN_Msk;

  /* Wait for HALTED bit to clear */
  while(axi_dma_dir->SR & AXI_DMA_SR_HALTED_Msk);

  gic_irq_enable(ddp->irq_id);
}

/** Stop an AXI DMA direction driver.
 *
 * \param ddp     Pointer to the axi_dma_dir_driver_t object.
 */
static void axi_dma_dir_stop(axi_dma_dir_driver_t *ddp)
{
  axi_dma_dir_t *axi_dma_dir = (axi_dma_dir_t *)ddp->axi_dma_dir;
  osalDbgAssert(axi_dma_dir != 0, "DMA dir not present");

  /* Clear ENABLE bit */
  axi_dma_dir->CR &= ~AXI_DMA_CR_RUN_Msk;

  /* Wait for HALTED bit to set */
  while(!(axi_dma_dir->SR & AXI_DMA_SR_HALTED_Msk));

  gic_irq_disable(ddp->irq_id);
}

/** Initialize interrupts for an AXI DMA direction driver.
 *
 * \param ddp     Pointer to the axi_dma_dir_driver_t object.
 */
static void interrupts_init(axi_dma_dir_driver_t *ddp)
{
  gic_handler_register(ddp->irq_id, axi_dma_irq_handler, ddp);
  gic_irq_sensitivity_set(ddp->irq_id, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(ddp->irq_id, ddp->irq_priority);
}

/** AXI DMA IRQ handler.
 *
 * \param context   Interrupt context (pointer to axi_dma_dir_driver_t).
 */
static void axi_dma_irq_handler(void *context)
{
  axi_dma_dir_driver_t *ddp = (axi_dma_dir_driver_t *)context;
  axi_dma_dir_t *axi_dma_dir = (axi_dma_dir_t *)ddp->axi_dma_dir;

  uint32_t sr = axi_dma_dir->SR;

  /* Clear interrupt flags */
  axi_dma_dir->SR = sr;

  /* COMPLETE */
  if (sr & AXI_DMA_INT_COMPLETE_Msk) {
    if (ddp->callback)
      ddp->callback(true);
  }

  /* ERROR */
  if (sr & AXI_DMA_INT_ERROR_Msk) {
    if (ddp->callback)
      ddp->callback(false);
  }
}

/** Initialize the AXI DMA module.
 */
void axi_dma_init(void)
{
#if ZYNQ7000_AXI_DMA_USE_AXI_DMA0 == TRUE
  #if ZYNQ7000_AXI_DMA0_MM2S_PRESENT == TRUE
    AXIDMADriver1.mm2s_driver.axi_dma_dir = &AXI_DMA0->MM2S;
    AXIDMADriver1.mm2s_driver.irq_id = IRQ_ID_AXI_DMA0_MM2S;
    AXIDMADriver1.mm2s_driver.irq_priority = ZYNQ7000_AXI_DMA0_MM2S_IRQ_PRIORITY;
    AXIDMADriver1.mm2s_driver.callback = 0;
    interrupts_init(&AXIDMADriver1.mm2s_driver);
  #else
    AXIDMADriver1.mm2s_driver.axi_dma_dir = 0;
  #endif

  #if ZYNQ7000_AXI_DMA0_S2MM_PRESENT == TRUE
    AXIDMADriver1.s2mm_driver.axi_dma_dir = &AXI_DMA0->S2MM;
    AXIDMADriver1.s2mm_driver.irq_id = IRQ_ID_AXI_DMA0_S2MM;
    AXIDMADriver1.s2mm_driver.irq_priority = ZYNQ7000_AXI_DMA0_S2MM_IRQ_PRIORITY;
    AXIDMADriver1.s2mm_driver.callback = 0;
    interrupts_init(&AXIDMADriver1.s2mm_driver);
  #else
    AXIDMADriver1.mm2s_driver.axi_dma_dir = 0;
  #endif
#endif

#if ZYNQ7000_AXI_DMA_USE_AXI_DMA1 == TRUE
  #if ZYNQ7000_AXI_DMA1_MM2S_PRESENT == TRUE
    AXIDMADriver2.mm2s_driver.axi_dma_dir = &AXI_DMA1->MM2S;
    AXIDMADriver2.mm2s_driver.irq_id = IRQ_ID_AXI_DMA1_MM2S;
    AXIDMADriver2.mm2s_driver.irq_priority = ZYNQ7000_AXI_DMA1_MM2S_IRQ_PRIORITY;
    AXIDMADriver2.mm2s_driver.callback = 0;
    interrupts_init(&AXIDMADriver2.mm2s_driver);
  #else
    AXIDMADriver2.mm2s_driver.axi_dma_dir = 0;
  #endif

  #if ZYNQ7000_AXI_DMA1_S2MM_PRESENT == TRUE
    AXIDMADriver2.s2mm_driver.axi_dma_dir = &AXI_DMA1->S2MM;
    AXIDMADriver2.s2mm_driver.irq_id = IRQ_ID_AXI_DMA1_S2MM;
    AXIDMADriver2.s2mm_driver.irq_priority = ZYNQ7000_AXI_DMA1_S2MM_IRQ_PRIORITY;
    AXIDMADriver2.s2mm_driver.callback = 0;
    interrupts_init(&AXIDMADriver2.s2mm_driver);
  #else
    AXIDMADriver2.s2mm_driver.axi_dma_dir = 0;
  #endif
#endif
}

/** Configure and start an AXI DMA peripheral.
 *
 * \param dp      Pointer to the AXIDMADriver object.
 */
void axi_dma_start(AXIDMADriver *dp)
{
  axi_dma_dir_t *mm2s = (axi_dma_dir_t *)dp->mm2s_driver.axi_dma_dir;
  axi_dma_dir_t *s2mm = (axi_dma_dir_t *)dp->s2mm_driver.axi_dma_dir;

  /* Note: issuing a reset on either MM2S or S2MM will
   * reset the entire peripheral. */
  if (mm2s)
    mm2s->CR = AXI_DMA_CR_RESET_Msk;
  else if (s2mm)
    s2mm->CR = AXI_DMA_CR_RESET_Msk;

  /* Wait for RESET bits to clear */
  if (mm2s)
    while(mm2s->CR & AXI_DMA_CR_RESET_Msk);
  if (s2mm)
    while(s2mm->CR & AXI_DMA_CR_RESET_Msk);

  if (mm2s)
    axi_dma_dir_start(&dp->mm2s_driver);
  if (s2mm)
    axi_dma_dir_start(&dp->s2mm_driver);
}

/** Deactivate an AXI DMA peripheral.
 *
 * \param dp      Pointer to the AXIDMADriver object.
 */
void axi_dma_stop(AXIDMADriver *dp)
{
  axi_dma_dir_t *mm2s = (axi_dma_dir_t *)dp->mm2s_driver.axi_dma_dir;
  axi_dma_dir_t *s2mm = (axi_dma_dir_t *)dp->s2mm_driver.axi_dma_dir;

  if (mm2s)
    axi_dma_dir_stop(&dp->mm2s_driver);
  if (s2mm)
    axi_dma_dir_stop(&dp->s2mm_driver);
}

/** Begin an AXI DMA write transfer.
 *
 * \param dp            Pointer to the AXIDMADriver object.
 * \param data          Data to be written.
 * \param data_length   Length of the data to be written.
 * \param callback      Callback to be executed on completion.
 *
 * \note The callback will be executed from interrupt context.
 */
void axi_dma_write_begin(AXIDMADriver *dp, const uint8_t *data,
                         uint32_t data_length, axi_dma_callback_t callback)
{
  axi_dma_dir_transfer_begin(&dp->mm2s_driver, data, data_length, callback);
}

/** Begin an AXI DMA read transfer.
 *
 * \param dp            Pointer to the AXIDMADriver object.
 * \param data          Data to be read.
 * \param data_length   Length of the data to be read.
 * \param callback      Callback to be executed on completion.
 *
 * \note The callback will be executed from interrupt context.
 */
void axi_dma_read_begin(AXIDMADriver *dp, uint8_t *data,
                        uint32_t data_length, axi_dma_callback_t callback)
{
  axi_dma_dir_transfer_begin(&dp->s2mm_driver, data, data_length, callback);
}
