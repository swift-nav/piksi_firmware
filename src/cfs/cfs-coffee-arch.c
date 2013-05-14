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
  printf("Coffee: write %u @ 0x%08lX\n", (unsigned int)size, COFFEE_START+offset);

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


