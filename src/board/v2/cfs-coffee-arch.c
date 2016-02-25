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

#include <stdio.h>

#include "cfs-coffee-arch.h"
#include "peripherals/stm_flash.h"

/** \addtogroup cfs
 * \{
 */

/** \defgroup cfs_arch Local Coffee configuration
 * \{
 */

/** Read from the Coffee filesystem area in STM flash.
 * \param buf Pointer to a buffer where the read values will be stored.
 * \param size Number of bytes to read.
 * \param offset Offset into the filesystem area to read from.
 */
void coffee_read(u8* buf, u32 size, u32 offset)
{
  for (u32 i=0; i<size; i++)
    buf[i] = ~((u8*)(COFFEE_START+offset))[i];
}

/** Write to the Coffee filesystem area in STM flash.
 * \param buf Pointer to a buffer containing the values to be written.
 * \param size Number of bytes to write.
 * \param offset Offset into the filesystem area to write to.
 */
void coffee_write(u8* buf, u32 size, u32 offset)
{
  flash_unlock();

  for (u32 i=0; i<size; i++)
    flash_program_byte(COFFEE_START+offset+i, ~buf[i]);

  flash_lock();
}

/** Erase sector of the Coffee filesystem area in STM flash.
 * Erases flash sectors of size \ref COFFEE_SECTOR_SIZE.
 * \param sector Sector number to erase, starting from zero.
 */
void coffee_erase(u8 sector)
{
  flash_unlock();
  stm_flash_erase_sector(sector+COFFEE_START_SECTOR);
  flash_lock();
}

/** \} */

/** \} */

