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

#ifndef SWIFTNAV_XADC_IF_REGS_H
#define SWIFTNAV_XADC_IF_REGS_H

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CFG;
  volatile uint32_t INT_STAT;
  volatile uint32_t INT_MASK;
  volatile uint32_t STAT;
  volatile uint32_t CMD_FIFO;
  volatile uint32_t DATA_FIFO;
  volatile uint32_t CTRL;
} xadc_if_t;

/* Bitfields */
#define XADC_IF_CFG_IGAP_Pos (0U)
#define XADC_IF_CFG_IGAP_Msk (0x1FU << XADC_IF_CFG_IGAP_Pos)

#define XADC_IF_CFG_TCKRATE_Pos (8U)
#define XADC_IF_CFG_TCKRATE_Msk (0x3U << XADC_IF_CFG_TCKRATE_Pos)
#define XADC_IF_CFG_TCKRATE_DIV2 (0x0U)
#define XADC_IF_CFG_TCKRATE_DIV4 (0x1U)
#define XADC_IF_CFG_TCKRATE_DIV8 (0x2U)
#define XADC_IF_CFG_TCKRATE_DIV16 (0x3U)

#define XADC_IF_CFG_REDGE_Pos (12U)
#define XADC_IF_CFG_REDGE_Msk (0x1U << XADC_IF_CFG_REDGE_Pos)
#define XADC_IF_CFG_REDGE_FALLING (0x0U)
#define XADC_IF_CFG_REDGE_RISING (0x1U)

#define XADC_IF_CFG_WEDGE_Pos (13U)
#define XADC_IF_CFG_WEDGE_Msk (0x1U << XADC_IF_CFG_WEDGE_Pos)
#define XADC_IF_CFG_WEDGE_FALLING (0x0U)
#define XADC_IF_CFG_WEDGE_RISING (0x1U)

#define XADC_IF_CFG_DFIFOTH_Pos (16U)
#define XADC_IF_CFG_DFIFOTH_Msk (0xFU << XADC_IF_CFG_DFIFOTH_Pos)

#define XADC_IF_CFG_CFIFOTH_Pos (20U)
#define XADC_IF_CFG_CFIFOTH_Msk (0xFU << XADC_IF_CFG_CFIFOTH_Pos)

#define XADC_IF_CFG_ENABLE_Pos (31U)
#define XADC_IF_CFG_ENABLE_Msk (0x1U << XADC_IF_CFG_ENABLE_Pos)


#define XADC_IF_INT_ALM0_Pos (0U)
#define XADC_IF_INT_ALM0_Msk (0x1U << XADC_IF_INT_ALM0_Pos)

#define XADC_IF_INT_OT_Pos (7U)
#define XADC_IF_INT_OT_Msk (0x1U << XADC_IF_INT_OT_Pos)

#define XADC_IF_INT_DFIFOGTH_Pos (8U)
#define XADC_IF_INT_DFIFOGTH_Msk (0x1U << XADC_IF_INT_DFIFOGTH_Pos)

#define XADC_IF_INT_CFIFO_LTH_Pos (9U)
#define XADC_IF_INT_CFIFO_LTH_Msk (0x1U << XADC_IF_INT_CFIFO_LTH_Pos)


#define XADC_IF_STAT_ALM_Pos (0U)
#define XADC_IF_STAT_ALM_Msk (0x7FU << XADC_IF_STAT_ALM_Pos)

#define XADC_IF_STAT_OT_Pos (7U)
#define XADC_IF_STAT_OT_Msk (0x1U << XADC_IF_STAT_OT_Pos)

#define XADC_IF_STAT_DFIFOE_Pos (8U)
#define XADC_IF_STAT_DFIFOE_Msk (0x1U << XADC_IF_STAT_DFIFOE_Pos)

#define XADC_IF_STAT_DFIFOF_Pos (9U)
#define XADC_IF_STAT_DFIFOF_Msk (0x1U << XADC_IF_STAT_DFIFOF_Pos)

#define XADC_IF_STAT_CFIFOE_Pos (10U)
#define XADC_IF_STAT_CFIFOE_Msk (0x1U << XADC_IF_STAT_CFIFOE_Pos)

#define XADC_IF_STAT_CFIFOF_Pos (11U)
#define XADC_IF_STAT_CFIFOF_Msk (0x1U << XADC_IF_STAT_CFIFOF_Pos)

#define XADC_IF_STAT_DFIFO_LVL_Pos (12U)
#define XADC_IF_STAT_DFIFO_LVL_Msk (0xFU << XADC_IF_STAT_DFIFO_LVL_Pos)

#define XADC_IF_STAT_CFIFO_LVL_Pos (16U)
#define XADC_IF_STAT_CFIFO_LVL_Msk (0xFU << XADC_IF_STAT_CFIFO_LVL_Pos)


#define XADC_IF_CTRL_RESET_Pos (4U)
#define XADC_IF_CTRL_RESET_Msk (0x1U << XADC_IF_CTRL_RESET_Pos)


#define XADC_IF_CMD(op, addr, data)   (((op) << 26) |                         \
                                       ((addr) << 16) |                       \
                                       ((data) << 0))
#define XADC_IF_OP_NOP    (0x0U)
#define XADC_IF_OP_READ   (0x1U)
#define XADC_IF_OP_WRITE  (0x2U)

/* Instances */
#define XADC_IF ((xadc_if_t *)0xF8007100U)

#endif /* SWIFTNAV_XADC_IF_REGS_H */
