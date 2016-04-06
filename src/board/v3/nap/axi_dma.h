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

#ifndef SWIFTNAV_AXI_DMA_H
#define SWIFTNAV_AXI_DMA_H

#include <stdint.h>
#include <stdbool.h>

#include <osal.h>
#include <board.h>
#include <halconf.h>

/* Note: callbacks are executed from interrupt context */
typedef void (*axi_dma_callback_t)(bool success);

typedef struct {
  void *axi_dma_dir;
  uint8_t irq_id;
  uint8_t irq_priority;
  axi_dma_callback_t callback;
} axi_dma_dir_driver_t;

typedef struct {
  axi_dma_dir_driver_t mm2s_driver;
  axi_dma_dir_driver_t s2mm_driver;
} AXIDMADriver;

#if (ZYNQ7000_AXI_DMA_USE_AXI_DMA0 == TRUE) && !defined(__DOXYGEN__)
extern AXIDMADriver AXIDMADriver1;
#endif

#if (ZYNQ7000_AXI_DMA_USE_AXI_DMA1 == TRUE) && !defined(__DOXYGEN__)
extern AXIDMADriver AXIDMADriver2;
#endif

void axi_dma_init(void);
void axi_dma_start(AXIDMADriver *dp);
void axi_dma_stop(AXIDMADriver *dp);
void axi_dma_write_begin(AXIDMADriver *dp, const uint8_t *data,
                         uint32_t data_length, axi_dma_callback_t callback);
void axi_dma_read_begin(AXIDMADriver *dp, uint8_t *data,
                        uint32_t data_length, axi_dma_callback_t callback);

#endif /* SWIFTNAV_AXI_DMA_H */
