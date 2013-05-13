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

extern u32 coffee_fs_area;
extern u32 ecoffee_fs_area;

#define COFFEE_START              coffee_fs_area

#define COFFEE_SIZE               (coffee_fs_area - ecoffee_fs_area)

/* The maximum filename length. */
#define COFFEE_NAME_LENGTH        8

#define COFFEE_MAX_OPEN_FILES     8

#define COFFEE_FD_SET_SIZE        8

#define COFFEE_MICRO_LOGS         1

#define COFFEE_DYN_SIZE           (4*COFFEE_PAGE_SIZE)

#define COFFEE_LOG_SIZE           (COFFEE_PAGE_SIZE/4)

#define COFFEE_LOG_TABLE_LIMIT    256


#define COFFEE_WRITE(buf, size, offset) \
  do { \
    flash_unlock(); \
    flash_program(COFFEE_START+(offset), (u8*)(buf), (size)); \
    flash_lock(); \
  } while (0)

#define COFFEE_READ(buf, size, offset) \
  memcpy((char *)(buf), (char *)(COFFEE_START+(offset)), (size))

#define COFFEE_ERASE(sector) \
  do { \
    flash_unlock(); \
    flash_erase_sector((sector), FLASH_CR_PROGRAM_X32); \
    flash_lock(); \
  } while (0)

#define coffee_page_t u16

int coffee_file_test(void);

#endif /* !COFFEE_ARCH_H */
