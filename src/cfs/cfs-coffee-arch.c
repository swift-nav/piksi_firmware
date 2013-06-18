/*
 * Copyright (C) 2013 Swift Navigation Inc.
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

#include "cfs/cfs-coffee-arch.h"

void coffee_read(u8* buf, u32 size, u32 offset)
{
  for (u32 i=0; i<size; i++)
    buf[i] = ~((u8*)(COFFEE_START+offset))[i];
}

void coffee_write(u8* buf, u32 size, u32 offset)
{
  flash_unlock();

  for (u32 i=0; i<size; i++)
    flash_program_byte(COFFEE_START+offset+i, ~buf[i]);

  flash_lock();
}

void coffee_erase(u8 sector)
{
  flash_unlock();
  flash_erase_sector(sector+COFFEE_START_SECTOR, FLASH_CR_PROGRAM_X32);
  flash_lock();
}


