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

#ifndef SWIFTNAV_NAP_REGS_H
#define SWIFTNAV_NAP_REGS_H

#include <stdint.h>

typedef struct {

} nap_trk_regs_t;

/* Registers */
typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CONTROL;
  volatile uint32_t IRQ;
  volatile uint32_t IRQ_ERROR;
  volatile uint32_t TIMING_COUNT;
  volatile uint32_t ACQ_STATUS;
  volatile uint32_t ACQ_CONTROL;
  volatile uint32_t ACQ_TIMING_COMPARE;
  volatile uint32_t ACQ_TIMING_SNAPSHOT;
  volatile uint32_t ACQ_START_SNAPSHOT;
  volatile uint32_t ACQ_FFT_CONFIG;
  volatile uint32_t TRK_CONTROL;
  volatile uint32_t TRK_IRQ;
  volatile uint32_t TRK_IRQ_ERROR;
  volatile uint32_t TRK_TIMING_COMPARE;
  volatile uint32_t TRK_TIMING_SNAPSHOT;
  volatile uint32_t FE_PINC[8];
  nap_trk_regs_t TRK_CH[0];
} nap_t;

/* Bitfields */
#define NAP_ACQ_CONTROL_DMA_INPUT_Pos (0U)
#define NAP_ACQ_CONTROL_DMA_INPUT_Msk (0x1U << NAP_ACQ_CONTROL_DMA_INPUT_Pos)
#define NAP_ACQ_CONTROL_DMA_INPUT_FFT (0U)
#define NAP_ACQ_CONTROL_DMA_INPUT_SAMPLE_GRABBER (1U)

#define NAP_ACQ_CONTROL_FFT_INPUT_Pos (1U)
#define NAP_ACQ_CONTROL_FFT_INPUT_Msk (0x1U << NAP_ACQ_CONTROL_FFT_INPUT_Pos)
#define NAP_ACQ_CONTROL_FFT_INPUT_DMA (0U)
#define NAP_ACQ_CONTROL_FFT_INPUT_FRONTEND (1U)

#define NAP_ACQ_CONTROL_RF_FE_Pos (2U)
#define NAP_ACQ_CONTROL_RF_FE_Msk (0x3U << NAP_ACQ_CONTROL_RF_FE_Pos)

#define NAP_ACQ_CONTROL_RF_FE_CH_Pos (4U)
#define NAP_ACQ_CONTROL_RF_FE_CH_Msk (0x1U << NAP_ACQ_CONTROL_RF_FE_CH_Pos)

#define NAP_ACQ_CONTROL_LENGTH_Pos (5U)
#define NAP_ACQ_CONTROL_LENGTH_Msk (0xFFFFFU << NAP_ACQ_CONTROL_LENGTH_Pos)


#define NAP_ACQ_FFT_CONFIG_DIR_Pos (0U)
#define NAP_ACQ_FFT_CONFIG_DIR_Msk (0x1U << NAP_ACQ_FFT_CONFIG_DIR_Pos)

#define NAP_ACQ_FFT_CONFIG_SCALE_Pos (1U)
#define NAP_ACQ_FFT_CONFIG_SCALE_Msk (0x3FFFFFFFU << NAP_ACQ_FFT_CONFIG_SCALE_Pos)

/* Instances */
#define NAP ((nap_t *)0x43C00000)

#endif /* SWIFTNAV_NAP_REGS_H */
