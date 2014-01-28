/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_STM_FLASH_H
#define SWIFTNAV_STM_FLASH_H

#include <libswiftnav/common.h>

/** \addtogroup stm_flash
 * \{ */

/* TODO : put this in libopencm3 */
#define STM_UNIQUE_ID_ADDR 0x1FFF7A10

#define STM_FLASH_N_SECTORS 12

#define STM_FLASH_MIN_ADDR 0x08000000
#define STM_FLASH_MAX_ADDR 0x080FFFFF

/** \} */

void stm_flash_lock_sector(u8 sector);
void stm_flash_unlock_sector(u8 sector);

void stm_flash_erase_sector(u8 sector);
void stm_flash_program(u32 address, u8 data[], u8 length);

#endif
