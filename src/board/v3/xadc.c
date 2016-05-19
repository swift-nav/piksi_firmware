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

#include "xadc.h"

#include <assert.h>
#include <math.h>

#include "xadc_if_regs.h"
#include "xadc_regs.h"

#include "zynq7000.h"
#include "gic.h"

#define TEMP_CRIT_UPPER_C 115.0f
#define TEMP_CRIT_LOWER_C  90.0f

#define TEMP_WARN_UPPER_C 100.0f
#define TEMP_WARN_LOWER_C  90.0f

#define TEMP_VAL(temp) \
            (((u32)roundf(((temp) + 273.15f) * 4096 / 509.375f) << 4) | 0b0011)

static u16 xadc_cmd(u8 op, u8 addr, u16 data);
static u16 xadc_read(u8 addr);
static void xadc_write(u8 reg, u16 data);
static void xadc_irq_handler(void *context);

/** Execute an XADC command.
 *
 * \param op     Type of operation to perform.
 * \param addr   XADC address to use.
 * \param data   Data to send.
 *
 * \return The received data.
 */
static u16 xadc_cmd(u8 op, u8 addr, u16 data)
{
  /* Write to CMD FIFO */
  XADC_IF->CMD_FIFO = XADC_IF_CMD(op, addr, data);

  /* Wait for CMD FIFO to be empty */
  u32 timeout = 10000;
  while (!(XADC_IF->STAT & XADC_IF_STAT_CFIFOE_Msk)) {
    if (--timeout == 0) {
      assert(!"XADC command timeout");
      break;
    }
  }

  /* Read from DATA FIFO */
  return XADC_IF->DATA_FIFO & 0xffff;
}

/** Execute an XADC read command.
 *
 * \param addr   XADC address to read from.
 *
 * \return The read data.
 */
static u16 xadc_read(u8 addr)
{
  /* Send READ command */
  xadc_cmd(XADC_IF_OP_READ, addr, 0);

  /* Send NOP command to get data */
  return xadc_cmd(XADC_IF_OP_NOP, 0, 0);
}

/** Execute an XADC write command.
 *
 * \param addr   XADC address to write to.
 * \param data   Data to write.
 */
static void xadc_write(u8 reg, u16 data)
{
  /* Send WRITE command */
  xadc_cmd(XADC_IF_OP_WRITE, reg, data);
}

/** XADC IRQ handler.
 *
 * \param context   Interrupt context (unused).
 */
static void xadc_irq_handler(void *context)
{
  (void)context;

  u32 int_stat = XADC_IF->INT_STAT;
  XADC_IF->INT_MASK |= int_stat;
  XADC_IF->INT_STAT = int_stat;
}

/** Initialize the XADC.
 */
void xadc_init(void)
{
  /* Reset interface */
  XADC_IF->CTRL |= XADC_IF_CTRL_RESET_Msk;
  XADC_IF->CTRL &= ~XADC_IF_CTRL_RESET_Msk;

  /* Write interface config register */
  XADC_IF->CFG = (20                            << XADC_IF_CFG_IGAP_Pos) |
                 (XADC_IF_CFG_TCKRATE_DIV4      << XADC_IF_CFG_TCKRATE_Pos) |
                 (XADC_IF_CFG_REDGE_RISING      << XADC_IF_CFG_REDGE_Pos) |
                 (XADC_IF_CFG_WEDGE_FALLING     << XADC_IF_CFG_WEDGE_Pos) |
                 (0                             << XADC_IF_CFG_DFIFOTH_Pos) |
                 (0                             << XADC_IF_CFG_CFIFOTH_Pos) |
                 (1                             << XADC_IF_CFG_ENABLE_Pos);

  /* Send RESET command */
  xadc_write(XADC_ADDR_RESET, 0);

  /* Send NOPs */
  for (u32 i=0; i<16; i++) {
    xadc_cmd(XADC_IF_OP_NOP, 0, 0);
  }

  /* Configure XADC */
  xadc_write(XADC_ADDR_ALARM_OT_UPPER, TEMP_VAL(TEMP_CRIT_UPPER_C));
  xadc_write(XADC_ADDR_ALARM_OT_LOWER, TEMP_VAL(TEMP_CRIT_LOWER_C));
  xadc_write(XADC_ADDR_ALARM_TEMP_UPPER, TEMP_VAL(TEMP_WARN_UPPER_C));
  xadc_write(XADC_ADDR_ALARM_TEMP_LOWER, TEMP_VAL(TEMP_WARN_LOWER_C));
  xadc_write(XADC_ADDR_CFG0, 0x0000);
  xadc_write(XADC_ADDR_CFG1, 0x8000);
  xadc_write(XADC_ADDR_CFG2, 0x1E00);

  /* Configure interrupts */
  XADC_IF->INT_MASK = ~(XADC_IF_INT_OT_Msk | XADC_IF_INT_ALM0_Msk);
  XADC_IF->INT_STAT = ~0;

  gic_handler_register(IRQ_ID_XACD, xadc_irq_handler, 0);
  gic_irq_sensitivity_set(IRQ_ID_XACD, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_XACD, 4);
  gic_irq_enable(IRQ_ID_XACD);
}

/** Get the most recent die temperature (C).
 */
float xadc_die_temp_get(void)
{
  return -273.15f + (float)xadc_read(XADC_ADDR_TEMPERATURE) * 503.975f / 65536;
}

/** Returns true if the die temperature has exceeded the warning limit.
 */
bool xadc_die_temp_warning(void)
{
  return (XADC_IF->INT_MASK & XADC_IF_INT_ALM0_Msk) ? true : false;
}

/** Returns true if the die temperature has exceeded the critical limit.
 */
bool xadc_die_temp_critical(void)
{
  return (XADC_IF->INT_MASK & XADC_IF_INT_OT_Msk) ? true : false;
}
