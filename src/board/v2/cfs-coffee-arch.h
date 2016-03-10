/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef CFS_COFFEE_ARCH_H
#define CFS_COFFEE_ARCH_H

#include <libswiftnav/common.h>

/** \addtogroup cfs_arch
 * \{
 */

/* Minimum reservation unit for Coffee. It can be changed by the user. */
#define COFFEE_PAGE_SIZE        256

/* Minimum erasable size, 128K for STM32F4. */
#define COFFEE_SECTOR_SIZE      (128*1024)

extern u32 _coffee_fs_area;
extern u32 _ecoffee_fs_area;

#define COFFEE_START           ((u32)&_coffee_fs_area)
#define COFFEE_START_SECTOR    8
#define COFFEE_SIZE            ((u32)&_ecoffee_fs_area - (u32)&_coffee_fs_area)
#define COFFEE_NAME_LENGTH     8 /* The maximum filename length. */
#define COFFEE_MAX_OPEN_FILES  8
#define COFFEE_FD_SET_SIZE     8
#define COFFEE_MICRO_LOGS      1
#define COFFEE_DYN_SIZE        (4*COFFEE_PAGE_SIZE)
#define COFFEE_LOG_SIZE        (8*COFFEE_PAGE_SIZE)
#define COFFEE_LOG_TABLE_LIMIT 256

void coffee_write(u8* buf, u32 size, u32 offset);
void coffee_read(u8* buf, u32 size, u32 offset);
void coffee_erase(u8 sector);

#define COFFEE_WRITE(buf, size, offset) coffee_write((u8*)buf, size, offset)
#define COFFEE_READ(buf, size, offset)  coffee_read((u8*)buf, size, offset)
#define COFFEE_ERASE(sector)            coffee_erase(sector)

typedef u16 coffee_page_t;

int coffee_file_test(void);

/** \} */

#endif /* !COFFEE_ARCH_H */

