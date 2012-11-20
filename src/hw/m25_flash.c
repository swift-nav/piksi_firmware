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

#include "spi.h"
#include "m25_flash.h"
#include "../debug.h"
#include <stdio.h>


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

void m25_read(u32 addr, u32 len, u8 *buff)
{
  spi_slave_select(SPI_SLAVE_FLASH);

//  spi_xfer(SPI_BUS_FLASH, M25_FAST_READ);
  spi_xfer(SPI_BUS_FLASH, M25_READ);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

//  spi_xfer(SPI_BUS_FLASH, 0x00); // Dummy byte


  for(u32 i=0; i < len; i++)
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


void flash_write_callback(u8 buff[]) {
  // Msg format:  u32 addr, u8 len, u8 data[]
  // Write a 256-byte page at a time.
  // If you start partway through a page, that's ok, but if you run off the end of the page
  //  it will wrap, rather than going on to the next page.

  u32 addr = *(u32 *)&buff[0];
  u8 len = *(u8 *)&buff[4];
  u8 *data = &buff[5];

//  printf("SPI Flash writing %d bytes to 0x%06X...", (int)len, (unsigned int)addr);
  m25_write_enable();
  m25_page_program(addr, len, data);
//  printf("ok\n");

  debug_send_msg(MSG_FLASH_COMPLETE,0,0);   // Report completion
}

void flash_read_callback(u8 buff[]) {
  // Msg format:  u32 addr, u32 len
  u32 addr, len;
  static char flash_data[16];

  addr = *(u32 *)&buff[0];
  len  = *(u32 *)&buff[4];

//  printf("SPI Flash reading %d bytes from 0x%06X:\n", (int)len, (unsigned int)addr);

  while (len) {
    u8 chunk_len = 16;
    if (len < 16) chunk_len = len;

    m25_read(addr, chunk_len, (u8 *)flash_data);

    printf("%08X:  ", (unsigned int)addr);

    for (u8 chunk_i = 0; chunk_i < chunk_len; chunk_i++)
      printf("%02X ", flash_data[chunk_i]);
/*
    printf("  ");

    for (u8 chunk_i = 0; chunk_i < chunk_len; chunk_i++)
      if (flash_data[chunk_i] >= 32)
        printf("%c", flash_data[chunk_i]);
      else
        printf(".");
*/

    printf("\n");

    len -= chunk_len;
    addr += chunk_len;
  }


}

void flash_erase_callback(u8 buff[] ) {
  // Msg format: u32 addr
  // Erases a 65536-byte sector.  Any address within the sector will work.

  u32 addr = *(u32*)buff;

//  printf("SPI Flash erasing 64KB from 0x%06X...", (unsigned int)addr & 0xFF0000);

  m25_write_enable();
  m25_sector_erase(addr);

//  printf("ok\n");

  debug_send_msg(MSG_FLASH_COMPLETE,0,0);   // Report completion

}

void m25_setup(void) {
  // Assumes spi_setup already called

  static msg_callbacks_node_t flash_write_node, flash_read_node, flash_erase_node;
  debug_register_callback(MSG_FLASH_WRITE, &flash_write_callback, &flash_write_node);
  debug_register_callback(MSG_FLASH_READ, &flash_read_callback,  &flash_read_node);
  debug_register_callback(MSG_FLASH_ERASE, &flash_erase_callback, &flash_erase_node);

/*
  u32 m25_id = m25_read_id();

  printf("SPI flash capacity = %02X, type = %02X, manufacturer = %02X\n", (char) (m25_id & 0xFF), (char)((m25_id >> 8) & 0xFF), (char)((m25_id >> 16) & 0xFF));
*/
}
