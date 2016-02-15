/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

#include <libsbp/flash.h>

#include "../error.h"
#include "../main.h"
#include "../flash.h"

#include "stm_flash.h"

void flash_erase_sector(uint8_t sector);
static void flash_program(u32 addr, const u8 *data, u8 len);
static void flash_unlock_option_bytes(void);

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
 * \param sector Number of the sector to lock.
 * \return Error code
 */
u8 stm_flash_lock_sector(u8 sector)
{
  /* Check that sector argument is valid. */
  if (sector >= STM_FLASH_N_SECTORS)
    return FLASH_INVALID_SECTOR;

  flash_unlock_option_bytes();
  while (FLASH->SR & FLASH_SR_BSY) ;
  FLASH->OPTCR &= ~(1 << (16+sector));
  FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
  while (FLASH->SR & FLASH_SR_BSY) ;

  return FLASH_OK;
}

/** Unlock a sector of the STM flash memory.
 * Locked sectors can not be erased or programmed.
 *
 * \param sector Number of the sector to unlock.
 * \return Error code
 */
u8 stm_flash_unlock_sector(u8 sector)
{
  /* Check that sector argument is valid. */
  if (sector >= STM_FLASH_N_SECTORS)
    return FLASH_INVALID_SECTOR;

  flash_unlock_option_bytes();
  while (FLASH->SR & FLASH_SR_BSY) ;
  FLASH->OPTCR |= (1 << (16+sector));
  FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
  while (FLASH->SR & FLASH_SR_BSY) ;

  return FLASH_OK;
}

/** Erase a sector of the STM flash
 * \param sector Number of the sector to erase.
 * \return Error code
 */
u8 stm_flash_erase_sector(u8 sector)
{
  /* Check that sector argument is valid. */
  if (sector >= STM_FLASH_N_SECTORS)
    return FLASH_INVALID_SECTOR;

  /* Erase sector.
   * See "PM0081 : STM32F40xxx and STM32F41xxx Flash programming manual"
   */
  flash_unlock();
  flash_erase_sector(sector);
  flash_lock();

  return FLASH_OK;
}

/** Program a set of addresses of the STM32F4 flash memory.
 * Note : sector containing addresses must be erased before addresses can be
 * programmed.
 *
 * \param address Starting address of set to program
 * \param data    Data to program addresses with
 * \param length  Length of set of addresses to program - counts up from
 *                starting address
 * \return Error code
 */
u8 stm_flash_program(u32 address, u8 data[], u8 length)
{
  /* Check that arguments are valid. */
  if (address > STM_FLASH_MAX_ADDR)
    return FLASH_INVALID_ADDR;
  if (address < STM_FLASH_MIN_ADDR)
    return FLASH_INVALID_ADDR;
  if (address+length-1 > STM_FLASH_MAX_ADDR)
    return FLASH_INVALID_RANGE;

  /* Program specified addresses with data */
  flash_unlock();
  flash_program(address, data, length);
  flash_lock();

  return FLASH_OK;
}

static void flash_program(u32 addr, const u8 *data, u8 len)
{
  while(--len)
    flash_program_byte(addr++, *data++);
}

void flash_program_byte(u32 addr, u8 data)
{
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~(3 << 8);
  FLASH->CR |= FLASH_CR_PG;
  *(u8*)(addr) = data;
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_PG;
}

void flash_unlock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xcdef89ab;
}

void flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_sector(uint8_t sector)
{
  while (FLASH->SR & FLASH_SR_BSY);

  FLASH->CR &= ~(0xF << 3);
  FLASH->CR |= (sector << 3) & 0x78;
  FLASH->CR |= FLASH_CR_SER;
  FLASH->CR |= FLASH_CR_STRT;

  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_SER;
  FLASH->CR &= ~(0xF << 3);
}

static void flash_unlock_option_bytes(void)
{
	FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
	FLASH->OPTKEYR = 0x08192a3b;
	FLASH->OPTKEYR = 0x4c5d6e7f;
}

/** \} */

/** \} */

