/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_FE_REGS_H
#define SWIFTNAV_NAP_FE_REGS_H

#include <stdint.h>

typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CONTROL;
  volatile uint32_t VERSION;
  volatile uint32_t IRQ;
  volatile uint32_t IRQ_ERROR;
  volatile uint32_t ACQ_STATUS;
  volatile uint32_t TRK_STATUS[4];
  volatile uint32_t BB_PINC[4];
} nap_fe_t;

/* Bitfields */
#define NAP_FE_STATUS_CLOCK_LOCKED_Pos (0U)
#define NAP_FE_STATUS_CLOCK_LOCKED_Msk (0x1U << NAP_FE_STATUS_CLOCK_LOCKED_Pos)

#define NAP_FE_STATUS_FRONTEND_CH_Pos (1U)
#define NAP_FE_STATUS_FRONTEND_CH_Msk (0xFU << NAP_FE_STATUS_FRONTEND_CH_Pos)

#define NAP_FE_CONTROL_VERSION_ADDR_Pos (0U)
#define NAP_FE_CONTROL_VERSION_ADDR_Msk (0xFU << NAP_FE_CONTROL_VERSION_ADDR_Pos)

/* Instances */
#define NAP_FE ((nap_fe_t *)0x43C10000)

#endif /* SWIFTNAV_NAP_FE_REGS_H */
