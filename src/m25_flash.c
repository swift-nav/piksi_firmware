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

#include "board/spi.h"
#include "m25_flash.h"

void m25_write_enable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WREN);
  spi_slave_deselect();
}

void m25_write_disable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WRDI);
  spi_slave_deselect();
}

u32 m25_read_id(void)
{
  u8 manf_id, mem_type, mem_capacity;

  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_RDID);
  manf_id = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  mem_type = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  mem_capacity = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  spi_slave_deselect();

  return mem_capacity | mem_type << 8 | manf_id << 16;
}

u8 m25_read_status(void)
{
  u8 sr;

  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_RDSR);
  sr = spi_xfer(SPI_BUS_FLASH, 0x00);
  spi_slave_deselect();
  return sr;
}

void m25_write_status(u8 sr)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WRSR);
  spi_xfer(SPI_BUS_FLASH, sr);
  spi_slave_deselect();
}

void m25_read(u32 addr, u32 len, u8 buff[])
{
  u32 i;

  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_FAST_READ);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

  spi_xfer(SPI_BUS_FLASH, 0x00); /* Dummy byte */

  for(i = 0; i < len; i++)
    buff[i] = spi_xfer(SPI_BUS_FLASH, 0x00);

  spi_slave_deselect();
}


void m25_page_program(u32 addr, u8 len, u8 buff[])
{
  u32 i;

  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_PP);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

  for(i = 0; i < len; i++)
    spi_xfer(SPI_BUS_FLASH, buff[i]);

  spi_slave_deselect();

  while(m25_read_status() & M25_SR_WIP);
}

void m25_subsector_erase(u32 addr)
{
  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_SSE);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

  spi_slave_deselect();

  while(m25_read_status() & M25_SR_WIP);
}

void m25_sector_erase(u32 addr)
{
  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_SE);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

  spi_slave_deselect();

  while(m25_read_status() & M25_SR_WIP);
}

void m25_bulk_erase(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_BE);

  spi_slave_deselect();

  while(m25_read_status() & M25_SR_WIP);
}

