/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/flash.h>

#include "stm_flash.h"
#include "../error.h"

/** \defgroup peripherals Peripherals
 * Functions to interact with the on-chip STM32F4 peripherals.
 *
 * \{ */

/** \defgroup stm_flash STM Flash
 * Callback functions to read, write, and erase the STM32F4 flash memory.
 *
 * \{ */

/** Lock a sector of the STM flash memory.
 * Locked sectors can not be erased or programmed.
 *
 * \param sector Number of the sector to lock (0-11).
 */
void stm_flash_lock_sector(u8 sector)
{
  /* Valid STM32F40 sectors are 0-11. */
  if (sector > 11)
    screaming_death("stm_flash_lock_sector received sector > 11 ");

  flash_unlock_option_bytes();
  while (FLASH_SR & FLASH_SR_BSY) ;
  FLASH_OPTCR &= ~(1 << (16+sector));
  FLASH_OPTCR |= FLASH_OPTCR_OPTSTRT;
  while (FLASH_SR & FLASH_SR_BSY) ;
}

/** Unlock a sector of the STM flash memory.
 * Locked sectors can not be erased or programmed.
 *
 * \param sector Number of the sector to unlock (0-11).
 */
void stm_flash_unlock_sector(u8 sector)
{
  /* Valid STM32F40 sectors are 0-11. */
  if (sector > 11)
    screaming_death("stm_flash_unlock_sector received sector > 11 ");

  flash_unlock_option_bytes();
  while (FLASH_SR & FLASH_SR_BSY) ;
  FLASH_OPTCR |= (1 << (16+sector));
  FLASH_OPTCR |= FLASH_OPTCR_OPTSTRT;
  while (FLASH_SR & FLASH_SR_BSY) ;
}

/** Erase a sector of the STM flash
 * \param sector Number of the sector to erase (0-11).
 */
void stm_flash_erase_sector(u8 sector)
{
  /* Valid STM32F40 sectors are 0-11. */
  if (sector > 11)
    screaming_death("stm_flash_erase_sector received sector > 11 ");

  /* Erase sector.
   * See "PM0081 : STM32F40xxx and STM32F41xxx Flash programming manual"
   */
  flash_unlock();
  flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
  flash_lock();
}

/** Program a set of addresses of the STM32F4 flash memory.
 * Note : sector containing addresses must be erased before addresses can be
 * programmed.
 *
 * \param address Starting address of set to program
 * \param data    Data to program addresses with
 * \param length  Length of set of addresses to program - counts up from
 *                starting address
 */
void stm_flash_program(u32 address, u8 data[], u8 length)
{
  /* Valid STM32F40 Flash addresses are 0x08000000 to 0x080FFFFF. */
  if (address > 0x080FFFFF)
    screaming_death("stm_flash_program received addr > 0x080FFFFF ");
  if (address < 0x08000000)
    screaming_death("stm_flash_program received addr < 0x08000000 ");
  if (address+length-1 > 0x080FFFFF)
    screaming_death("stm_flash_program received addr+length+1 > 0x080FFFFF ");
  if (length > 128)
    screaming_death("stm_flash_program received length > 128 ");

  /* Program specified addresses with data */
  flash_unlock();
  flash_program(address, data, length);
  flash_lock();
}

/** \} */

/** \} */

