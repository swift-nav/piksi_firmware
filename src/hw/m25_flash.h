/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
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

#ifndef SWIFTNAV_M25_FLASH_H
#define SWIFTNAV_M25_FLASH_H

#include <libopencm3/cm3/common.h>

#define M25_WREN 0x06
#define M25_WRDI 0x04
#define M25_RDID 0x9F
#define M25_RDSR 0x05
#define M25_WRSR 0x01
#define M25_READ 0x03
#define M25_FAST_READ 0x0B
#define M25_PP 0x02
#define M25_SSE 0x20
#define M25_SE 0xD8
#define M25_BE 0xC7

#define M25_SR_SRWD (1<<7)
#define M25_SR_TB   (1<<5)
#define M25_SR_BP2  (1<<4)
#define M25_SR_BP1  (1<<3)
#define M25_SR_BP0  (1<<2)
#define M25_SR_WEL  (1<<1)
#define M25_SR_WIP  (1<<0)

void m25_write_enable(void);
void m25_write_disable(void);
u32 m25_read_id(void);
u8 m25_read_status(void);
void m25_write_status(u8 sr);
void m25_read(u32 addr, u32 len, u8 buff[]);
void m25_page_program(u32 addr, u8 len, u8 buff[]);
void m25_subsector_erase(u32 addr);
void m25_sector_erase(u32 addr);
void m25_bulk_erase(void);

void m25_setup(void);

#endif

