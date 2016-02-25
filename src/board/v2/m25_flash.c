/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#include "../error.h"
#include "../peripherals/spi_wrapper.h"
#include "../sbp.h"
#include "../flash.h"
#include "m25_flash.h"
#include "main.h"

/** \addtogroup board
 * \{ */

/** \defgroup m25 M25Pxx Flash
 * Interface to the M25Pxx FPGA configuration flash.
 * \{ */

/** Send "write enable" command to the flash. */
void m25_write_enable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_slave_xfer(SPI_SLAVE_FLASH, M25_WREN);
  spi_slave_deselect(SPI_SLAVE_FLASH);
}

/** Send "write disable" command to the flash. */
void m25_write_disable(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_slave_xfer(SPI_SLAVE_FLASH, M25_WRDI);
  spi_slave_deselect(SPI_SLAVE_FLASH);
}

/** Read the manufacturer and device identification out of the flash.
 * \param man_id   JEDEC manufacturer ID
 * \param mem_type Memory type
 * \param mem_type Memory capacity
 */
void m25_read_id(u8 *man_id, u8 *mem_type, u8 *mem_cap)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_slave_xfer(SPI_SLAVE_FLASH, M25_RDID);
  *man_id = (u8)spi_slave_xfer(SPI_SLAVE_FLASH, 0x00);
  *mem_type = (u8)spi_slave_xfer(SPI_SLAVE_FLASH, 0x00);
  *mem_cap = (u8)spi_slave_xfer(SPI_SLAVE_FLASH, 0x00);
  spi_slave_deselect(SPI_SLAVE_FLASH);
}

/** Read the status register out of the flash.
 * \return 8-bit value read from flash status register.
 */
u8 m25_read_status(void)
{
  u8 sr;

  spi_slave_select(SPI_SLAVE_FLASH);
  spi_slave_xfer(SPI_SLAVE_FLASH, M25_RDSR);
  sr = spi_slave_xfer(SPI_SLAVE_FLASH, 0x00);
  spi_slave_deselect(SPI_SLAVE_FLASH);

  return sr;
}

/** Write to flash status register.
 * Note : m25_write_enable() must be called before this function is called.
 * \param sr 8-bit value to write to flash status register.
 */
void m25_write_status(u8 sr)
{
  spi_slave_select(SPI_SLAVE_FLASH);
  spi_slave_xfer(SPI_SLAVE_FLASH, M25_WRSR);
  spi_slave_xfer(SPI_SLAVE_FLASH, sr);
  spi_slave_deselect(SPI_SLAVE_FLASH);
}

/** Read data from flash memory.
 * \todo Use fast read command instead of regular?
 * \param addr Starting address to read from
 * \param len Number of addresses to read
 * \param buff Array to write bytes read from flash to
 * \return Error code
 */
u8 m25_read(u32 addr, u8 buff[], u32 len)
{
  /* Check that address range to be written is valid. */
  if (addr > M25_MAX_ADDR)
    return FLASH_INVALID_ADDR;
  if (addr+len-1 > M25_MAX_ADDR)
    return FLASH_INVALID_RANGE;

  spi_slave_select(SPI_SLAVE_FLASH);

  spi_slave_xfer(SPI_SLAVE_FLASH, M25_READ);

  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 16) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 8) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, addr & 0xFF);

  for (u32 i = 0; i < len; i++)
    buff[i] = spi_slave_xfer(SPI_SLAVE_FLASH, 0x00);

  spi_slave_deselect(SPI_SLAVE_FLASH);

  return FLASH_OK;
}

/** Program a page of the flash.
 * Programs selected bits from 1 to 0. If the write will cross a page
 * boundary, the device will hang and report an error.
 *
 * \param addr Starting address to write to
 * \param len  Number of addresses to write
 * \param buff Array of bytes to write to flash
 * \return Error code
 */
u8 m25_page_program(u32 addr, u8 buff[], u8 len)
{
  /* Check that address range to be written is valid. */
  if (addr > M25_MAX_ADDR)
    return FLASH_INVALID_ADDR;

  /* Check if page boundary is crossed. */
  if (addr>>8 < (addr+len-1)>>8)
    return FLASH_INVALID_RANGE;

  u32 i;

  spi_slave_select(SPI_SLAVE_FLASH);

  spi_slave_xfer(SPI_SLAVE_FLASH, M25_PP);

  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 16) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 8) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, addr & 0xFF);

  for (i = 0; i < len; i++)
    spi_slave_xfer(SPI_SLAVE_FLASH, buff[i]);

  spi_slave_deselect(SPI_SLAVE_FLASH);

  while (m25_read_status() & M25_SR_WIP) ;

  return FLASH_OK;
}

/** Erase a sector of the flash.
 * Erases all memory addresses in a sector to 0xFF. Any address in the sector
 * can be passed.
 * \param addr Address inside sector to erase.
 * \return Error code
 */
u8 m25_sector_erase(u32 addr)
{
  /* Check that addr argument is valid. */
  if (addr > M25_MAX_ADDR)
    return FLASH_INVALID_ADDR;

  spi_slave_select(SPI_SLAVE_FLASH);

  spi_slave_xfer(SPI_SLAVE_FLASH, M25_SE);

  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 16) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, (addr >> 8) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FLASH, addr & 0xFF);

  spi_slave_deselect(SPI_SLAVE_FLASH);

  while (m25_read_status() & M25_SR_WIP) ;

  return FLASH_OK;
}

/** Erase the entire flash.
 * Erases all memory addresses in the flash to 0xFF.
 */
void m25_bulk_erase(void)
{
  spi_slave_select(SPI_SLAVE_FLASH);

  spi_slave_xfer(SPI_SLAVE_FLASH, M25_BE);

  spi_slave_deselect(SPI_SLAVE_FLASH);

  while (m25_read_status() & M25_SR_WIP) ;
}

/** \} */

/** \} */

