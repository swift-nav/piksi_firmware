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
#include <string.h>

#include "cfs-coffee-arch.h"

u8 _coffee_fs_area[2048];

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
  memcpy(buf, (void*)(COFFEE_START+offset), size);
}

/** Write to the Coffee filesystem area in STM flash.
 * \param buf Pointer to a buffer containing the values to be written.
 * \param size Number of bytes to write.
 * \param offset Offset into the filesystem area to write to.
 */
void coffee_write(const u8* buf, u32 size, u32 offset)
{
  memcpy((void*)(COFFEE_START+offset), buf, size);
}

/** Erase sector of the Coffee filesystem area in STM flash.
 * Erases flash sectors of size \ref COFFEE_SECTOR_SIZE.
 * \param sector Sector number to erase, starting from zero.
 */
void coffee_erase(u8 sector)
{
  memset((void*)COFFEE_START+(sector*1024), 0, 1024);
}

/** \} */

/** \} */

