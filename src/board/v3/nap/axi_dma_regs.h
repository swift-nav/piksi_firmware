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

#ifndef SWIFTNAV_AXI_DMA_REGS_H
#define SWIFTNAV_AXI_DMA_REGS_H

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t RESERVED0[4];
  volatile uint32_t ADDR_LSB;
  volatile uint32_t ADDR_MSB;
  volatile uint32_t RESERVED1[2];
  volatile uint32_t LENGTH;
  volatile uint32_t RESERVED2;
} axi_dma_dir_t;

typedef struct {
  axi_dma_dir_t MM2S;
  axi_dma_dir_t S2MM;
} axi_dma_t;

/* Bitfields */
#define AXI_DMA_CR_RUN_Pos (0U)
#define AXI_DMA_CR_RUN_Msk (0x1U << AXI_DMA_CR_RUN_Pos)

#define AXI_DMA_CR_RESET_Pos (2U)
#define AXI_DMA_CR_RESET_Msk (0x1U << AXI_DMA_CR_RESET_Pos)


#define AXI_DMA_SR_HALTED_Pos (0U)
#define AXI_DMA_SR_HALTED_Msk (0x1U << AXI_DMA_SR_HALTED_Pos)

#define AXI_DMA_SR_IDLE_Pos (1U)
#define AXI_DMA_SR_IDLE_Msk (0x1U << AXI_DMA_SR_IDLE_Pos)

#define AXI_DMA_SR_SLVERR_Pos (5U)
#define AXI_DMA_SR_SLVERR_Msk (0x1U << AXI_DMA_SR_SLVERR_Pos)

#define AXI_DMA_SR_DECERR_Pos (6U)
#define AXI_DMA_SR_DECERR_Msk (0x1U << AXI_DMA_SR_DECERR_Pos)


#define AXI_DMA_INT_COMPLETE_Pos (12U)
#define AXI_DMA_INT_COMPLETE_Msk (0x1U << AXI_DMA_INT_COMPLETE_Pos)

#define AXI_DMA_INT_ERROR_Pos (14U)
#define AXI_DMA_INT_ERROR_Msk (0x1U << AXI_DMA_INT_ERROR_Pos)

/* Instances */
#define AXI_DMA0 ((axi_dma_t *)0x40400000)
#define IRQ_ID_AXI_DMA0_MM2S IRQ_ID_FPGA0
#define IRQ_ID_AXI_DMA0_S2MM IRQ_ID_FPGA1

#endif /* SWIFTNAV_AXI_DMA_REGS_H */
