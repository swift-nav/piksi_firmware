/*
 * Copyright (C) 2013 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CFS_COFFEE_ARCH_H
#define CFS_COFFEE_ARCH_H

//#include "contiki-conf.h"

#include <libswiftnav/common.h>
#include <libopencm3/stm32/f4/flash.h>

/* Minimum reservation unit for Coffee. It can be changed by the user. */
#define COFFEE_PAGE_SIZE          256

/* Minimum erasable size, 128K for STM32F4. */
#define COFFEE_SECTOR_SIZE        (128*1024)

extern u32 _coffee_fs_area;
extern u32 _ecoffee_fs_area;

#define COFFEE_START              ((u32)&_coffee_fs_area)

#define COFFEE_START_SECTOR       6

#define COFFEE_SIZE               ((u32)&_ecoffee_fs_area - (u32)&_coffee_fs_area)

/* The maximum filename length. */
#define COFFEE_NAME_LENGTH        8

#define COFFEE_MAX_OPEN_FILES     8

#define COFFEE_FD_SET_SIZE        8

#define COFFEE_MICRO_LOGS         1

#define COFFEE_DYN_SIZE           (4*COFFEE_PAGE_SIZE)

#define COFFEE_LOG_SIZE           (COFFEE_PAGE_SIZE/4)

#define COFFEE_LOG_TABLE_LIMIT    256

void coffee_write(u8* buf, u32 size, u32 offset);
void coffee_read(u8* buf, u32 size, u32 offset);
void coffee_erase(u8 sector);

#define COFFEE_WRITE(buf, size, offset) coffee_write((u8*)buf, size, offset)

#define COFFEE_READ(buf, size, offset) coffee_read((u8*)buf, size, offset)

#define COFFEE_ERASE(sector) coffee_erase(sector)

typedef u16 coffee_page_t;

int coffee_file_test(void);

#endif /* !COFFEE_ARCH_H */

