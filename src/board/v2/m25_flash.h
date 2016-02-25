/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_M25_FLASH_H
#define SWIFTNAV_M25_FLASH_H

#include <libswiftnav/common.h>

/** \addtogroup m25
 * \{ */

#define M25_WREN      0x06
#define M25_WRDI      0x04
#define M25_RDID      0x9F
#define M25_RDSR      0x05
#define M25_WRSR      0x01
#define M25_READ      0x03
#define M25_FAST_READ 0x0B
#define M25_PP        0x02
#define M25_SE        0xD8
#define M25_BE        0xC7

#define M25_SR_SRWD (1 << 7)  /**< Status Register: SR Write Protect Bit */
#define M25_SR_BP2  (1 << 4)  /**< Status Register: Block Protect 2 Bit */
#define M25_SR_BP1  (1 << 3)  /**< Status Register: Block Protect 1 Bit */
#define M25_SR_BP0  (1 << 2)  /**< Status Register: Block Protect 0 Bit */
#define M25_SR_WEL  (1 << 1)  /**< Status Register: Write Enable Latch Bit */
#define M25_SR_WIP  (1 << 0)  /**< Status Register: Write In Progress Bit */

#define M25_MAX_ADDR 0xFFFFF

/** \} */

void m25_write_enable(void);
void m25_write_disable(void);
void m25_read_id(u8 *man_id, u8 *mem_type, u8 *mem_cap);
u8 m25_read_status(void);
void m25_write_status(u8 sr);
u8 m25_read(u32 addr, u8 buff[], u32 len);
u8 m25_page_program(u32 addr, u8 buff[], u8 len);
u8 m25_sector_erase(u32 addr);
void m25_bulk_erase(void);

#endif /* SWIFTNAV_M25_FLASH_H */

