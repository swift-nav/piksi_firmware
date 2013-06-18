/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
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

/** Send "write enable" command to the flash. */
void m25_write_enable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WREN);
  spi_slave_deselect();
}

/** Send "write disable" command to the flash. */
void m25_write_disable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WRDI);
  spi_slave_deselect();
}

/** Read the manufacturer and device identification out of the flash.
 * \param man_id   Pointer to u8 where JEDEC manufacturer ID will be stored.
 * \param mem_type Pointer to u8 where memory type will be stored.
 * \param mem_type Pointer to u8 where memory capacity will be stored.
 */
void m25_read_id(u8 *man_id, u8 *mem_type, u8 *mem_cap)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_RDID);
  *man_id = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  *mem_type = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  *mem_cap = (u8)spi_xfer(SPI_BUS_FLASH, 0x00);
  spi_slave_deselect();
}

/** Read the status register out of the flash.
 * \return 8-bit value read from flash status register.
 */
u8 m25_read_status(void)
{
  u8 sr;

  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_RDSR);
  sr = spi_xfer(SPI_BUS_FLASH, 0x00);
  spi_slave_deselect();
  return sr;
}

/** Write to flash status register.
 * Note : m25_write_enable() must be called before this function is called.
 * \param sr 8-bit value to write to flash status register.
 */
void m25_write_status(u8 sr)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_xfer(SPI_BUS_FLASH, M25_WRSR);
  spi_xfer(SPI_BUS_FLASH, sr);
  spi_slave_deselect();
}

/** Read data from flash memory.
 * \param addr Starting address to read from
 * \param len Number of addresses to read
 * \param buff Array to write bytes read from flash to
 */
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

/** Program a page of the flash.
 * Programs selected bits from 1 to 0. If addr is greater than the starting
 * address of the page and len is greater than the page length, the written
 * data that goes over the end of the page will wrap to the beginning of the
 * page.
 * \param addr Starting address to write to
 * \param len Number of addresses to write
 * \param buff Array of bytes to write to flash
 */
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

/** Erase a sector of the flash.
 * Erases all memory addresses in sector to 0xFF. Any address in the sector can
 * be passed.
 * \param addr Address inside sector to erase.
 */
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

/** Erase the entire flash.
 * Erases all memory addresses in the flash to 0xFF.
 */
void m25_bulk_erase(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);

  spi_xfer(SPI_BUS_FLASH, M25_BE);

  spi_slave_deselect();

  while(m25_read_status() & M25_SR_WIP);
}

/** Callback to program a set of addresses of the flash.
 * Note : sector containing addresses must be erased before addresses can be
 * programmed.
 *
 * \param buff Array of u8 (length >= 6) :
 *             - [0:3]   starting address of set to program
 *             - [4]     length of set of addresses to program - counts up
 *                       from starting address
 *             - [5:end] data to program addresses with
 */
void m25_flash_program_callback(u8 buff[])
{
  u32 addr = *(u32 *)&buff[0];
  u8 len = *(u8 *)&buff[4];
  u8 *data = &buff[5];

  m25_write_enable();
  m25_page_program(addr, len, data);

  /* Keep trying to send message until it makes it into the buffer. */
  while(sbp_send_msg(MSG_M25_FLASH_DONE, 0, 0));
}

/** Callback to read a set of addresses from the flash.
 *
 * \param buff Array of u8 (length 5) :
 *             - [0:3] starting address of set to read
 *             - [4]   length of set of addresses to read - counts up from
 *                     starting address
 */
void m25_flash_read_callback(u8 buff[])
{
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

    /* Keep trying to send message until it makes it into the buffer. */
    while(sbp_send_msg(MSG_M25_FLASH_READ,5 + chunk_len,callback_data));

    len -= chunk_len;
    addr += chunk_len;
  }
}

/** Callback to erase a sector of the flash.
 * \param buff Array of u8 (length 1) :
 *             - [0] flash sector number to erase.
 */
void m25_flash_erase_callback(u8 buff[])
{
  u8 sector = buff[0];
  u32 addr = ((u32)sector) << 16;

  m25_write_enable();
  m25_sector_erase(addr);

  /* Keep trying to send message until it makes it into the buffer. */
  while(sbp_send_msg(MSG_M25_FLASH_DONE, 0, 0));
}

/** Setup the M25 flash callback functions.
 * Note : the SPI2 bus must be setup and uncontested for these callbacks to
 * properly communicate with the M25 flash.
 */
void m25_setup(void)
{
  /* Assumes SPI bus already setup. */
  static msg_callbacks_node_t m25_flash_program_node;
  static msg_callbacks_node_t m25_flash_read_node;
  static msg_callbacks_node_t m25_flash_erase_node;

  sbp_register_callback(
    MSG_M25_FLASH_WRITE,
    &m25_flash_program_callback,
    &m25_flash_program_node
  );
  sbp_register_callback(
    MSG_M25_FLASH_READ,
    &m25_flash_read_callback,
    &m25_flash_read_node
  );
  sbp_register_callback(
    MSG_M25_FLASH_ERASE,
    &m25_flash_erase_callback,
    &m25_flash_erase_node
  );
}

/** \} */

/** \} */
