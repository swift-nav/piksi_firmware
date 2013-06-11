/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 * Copyright (C) 2013 Colin Beighley <colinbeighley@gmail.com>
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
#include <libopencm3/stm32/f4/flash.h>

#include "stm_flash.h"
#include "../sbp.h"
#include "../error.h"

/** \defgroup peripherals Peripherals
 * Functions to interact with the on-chip STM32F4 peripherals.
 *
 * \{ */

/** \defgroup stm_flash STM Flash
 * Callback functions to read, write, and erase the STM32F4 flash memory.
 *
 * \{ */

/** Callback to erase a sector of the STM32F4 flash memory.
 *
 * \param buff Array of u8 (length 1) :
 * <ul>
 *   <li> [0] flash sector number to erase.
 * </ul>
 */
void stm_flash_erase_sector_callback(u8 buff[])
{
  /* See "PM0081 : STM32F40xxx and STM32F41xxx Flash programming manual" */
  u8 sector = buff[0];

  /* Check to make sure the sector to be erased is from 0-11,
   * and complain if it isn't. */
  if (sector > 11)
    speaking_death("flash_erase_callback received sector > 11\n");

  /* Erase sector. */
  flash_unlock();
  flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
  flash_lock();

  /* Send message back to PC to signal operation is finished */
  sbp_send_msg(MSG_STM_FLASH_DONE, 0, 0);
}

/** Callback to program a set of addresses of the STM32F4 flash memory.
 * Note : sector containing addresses must be erased before addresses can be
 * programmed.
 *
 * \param buff Array of u8 (length >= 6) :
 *             - [0:3]   starting address of set to program
 *             - [4]     length of set of addresses to program - counts up
 *                       from starting address
 *             - [5:end] data to program addresses with
 */
void stm_flash_program_callback(u8 buff[])
{
  /* TODO : Add check to restrict addresses that can be programmed? */
  u32 address = *(u32 *)&buff[0];
  u8 length = buff[4];
  u8 *data = &buff[5];

  /* Program specified addresses with data */
  flash_unlock();
  flash_program(address, data, length);
  flash_lock();

  /* Send message back to PC to signal operation is finished */
  sbp_send_msg(MSG_STM_FLASH_DONE, 0, 0);
}

/** Callback to read a set of addresses of the STM32F4 flash memory.
 *
 * \param buff Array of u8 (length 5) :
 *             - [0:3] starting address of set to read
 *             - [4]   length of set of addresses to read - counts up from
 *                     starting address
 */
void stm_flash_read_callback(u8 buff[])
{
  u32 address = *(u32 *)&buff[0];
  u8 length = buff[4];

  u8 callback_data[length+5];
  /* Put address and length in array */
  callback_data[0] = buff[0];
  callback_data[1] = buff[1];
  callback_data[2] = buff[2];
  callback_data[3] = buff[3];
  callback_data[4] = buff[4];
  /* Copy data from addresses into array */
  for (u16 i=0; i<length; i++){
    callback_data[5+i] = *(u8 *)(address+i);
  }

  /* If sending message fails (buffer is full), keep trying until successful */
  while(sbp_send_msg(MSG_STM_FLASH_READ, length+5, callback_data));
}

/** Callback to read STM32F4's hardcoded unique ID.
 * Sends STM32F4 unique ID (12 bytes) back to host.
 */
void stm_unique_id_callback()
{
  sbp_send_msg(MSG_STM_UNIQUE_ID,12,(u8 *)STM_UNIQUE_ID_ADDR);
}

/** Setup STM flash callbacks. */
void stm_flash_callbacks_setup()
{
  /*
   * Create message callbacks node types to add to SBP callback
   * linked list for each flash callback defined above.
   */
  static msg_callbacks_node_t stm_flash_erase_sector_node;
  static msg_callbacks_node_t stm_flash_read_node;
  static msg_callbacks_node_t stm_flash_program_node;
  static msg_callbacks_node_t stm_unique_id_node;

  /* Insert callbacks in SBP callback linked list so they can be called. */
  sbp_register_callback(MSG_STM_FLASH_ERASE,
                          &stm_flash_erase_sector_callback,
                          &stm_flash_erase_sector_node);
  sbp_register_callback(MSG_STM_FLASH_READ,
                          &stm_flash_read_callback,
                          &stm_flash_read_node);
  sbp_register_callback(MSG_STM_FLASH_WRITE,
                          &stm_flash_program_callback,
                          &stm_flash_program_node);
  sbp_register_callback(MSG_STM_UNIQUE_ID,
                          &stm_unique_id_callback,
                          &stm_unique_id_node);
}

/** \} */

/** \} */
