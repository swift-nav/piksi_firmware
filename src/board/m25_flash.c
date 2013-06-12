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

#include <stdio.h>

#include "m25_flash.h"
#include "../sbp.h"
#include "../error.h"
#include "../peripherals/spi.h"
#include "../peripherals/usart.h"

/** \addtogroup board
 * \{ */

/** \defgroup m25 M25P80 Flash
 * Interface to read, write, and erase sectors of the M25P80 FPGA
 * configuration flash.
 * \{ */

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

  spi_xfer(SPI_BUS_FLASH, M25_READ);

  spi_xfer(SPI_BUS_FLASH, (addr >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, (addr >> 8) & 0xFF);
  spi_xfer(SPI_BUS_FLASH, addr & 0xFF);

  for(u32 i=0; i < len; i++)
    buff[i] = spi_xfer(SPI_BUS_FLASH, 0x00);

  spi_slave_deselect();
}


void m25_page_program(u32 addr, u8 len, u8 buff[])
{
  /* TODO: check for page boundary crossing. */
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

void flash_write_callback(u8 buff[])
{
  /* Msg format:  u32 addr, u8 len, u8 data[]
   * Write a 256-byte page at a time.
   * If you start partway through a page, that's ok, but if you run off the end
   * of the page it will wrap, rather than going on to the next page. */

  u32 addr = *(u32 *)&buff[0];
  u8 len = *(u8 *)&buff[4];
  u8 *data = &buff[5];

  m25_write_enable();
  m25_page_program(addr, len, data);

  sbp_send_msg(MSG_M25_FLASH_DONE, 0, 0);
}

void flash_read_callback(u8 buff[])
{
  /* Msg format:  u32 addr, u8 len */

  u32 addr, len;
  static char flash_data[M25_READ_SIZE];

  addr = *(u32 *)&buff[0];
  len = buff[4];
  u8 callback_data[M25_READ_SIZE+5];

  u8 chunk_len;
  while (len > 0) {
    chunk_len = (len < M25_READ_SIZE) ? len : M25_READ_SIZE;

    m25_read(addr, chunk_len, (u8 *)flash_data);

    /* Pack data for read callback back to host.
     * 3 bytes starting address, 1 byte length, chunk_len byes data */
    callback_data[0] = (addr >> 24) & 0xFF;
    callback_data[1] = (addr >> 16) & 0xFF;
    callback_data[2] = (addr >> 8) & 0xFF;
    callback_data[3] = (addr >> 0) & 0xFF;
    callback_data[4] = chunk_len;

    for (u8 i=0; i<chunk_len; i++) {
      callback_data[i+5] = flash_data[i];
    }

    /* Keep trying to send message until we succeed. */
    while(sbp_send_msg(MSG_M25_FLASH_READ,5 + chunk_len,callback_data));

    len -= chunk_len;
    addr += chunk_len;
  }
}

void flash_erase_callback(u8 buff[])
{
  /* Msg format: u8 sector. Erases one of the 16 sectors in the flash. */

  u8 sector = buff[0];
  u32 addr = ((u32)sector) << 16;

  m25_write_enable();
  m25_sector_erase(addr);

  sbp_send_msg(MSG_M25_FLASH_DONE, 0, 0);
}

void m25_setup(void)
{
  /* Assumes SPI bus already setup. */
  static msg_callbacks_node_t flash_write_node;
  static msg_callbacks_node_t flash_read_node;
  static msg_callbacks_node_t flash_erase_node;

  sbp_register_callback(
    MSG_M25_FLASH_WRITE,
    &flash_write_callback,
    &flash_write_node
  );
  sbp_register_callback(
    MSG_M25_FLASH_READ,
    &flash_read_callback,
    &flash_read_node
  );
  sbp_register_callback(
    MSG_M25_FLASH_ERASE,
    &flash_erase_callback,
    &flash_erase_node
  );
}

/** \} */

/** \} */
