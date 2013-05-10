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
#include "../debug.h"
#include "../error.h"

void stm_flash_erase_sector_callback(u8 buff[]){
  /* Msg format : 1 byte, sector number to erase (0-11)
   * See "PM0081 : STM32F40xxx and STM32F41xxx Flash programming manual" */
  u8 sector = buff[0];

  /* Check to make sure the sector to be erased is from 0-11,
   * and complain if it isn't */
  if (sector > 11)
    speaking_death("flash_erase_callback received sector > 11\n");

  /* Erase sector
   * First argument is u32 that gets OR'd into the FLASH CR register,
   *   bits [6:3] of FLASH CR are sector to erase.
   *   TODO : interface for this could be better, maybe this should
   *   be changed in libopencm3
   * Second argument is number of bytes to erase in parallel, shifted by 8
   *   to be OR'd in at the appropriate place in the FLASH CR register [9:8].
   *   TODO : the interface for this could also be better in libopencm3,
   *   and could use some #defines
   */
  flash_unlock();
  flash_erase_sector(((sector << 3) & 0x78),((u32)2 << 8) & 0x00000300);
  flash_lock();

  /* Send message back to PC to signal operation is finished */
  debug_send_msg(MSG_STM_FLASH_COMPLETE,0,0);
}

void stm_flash_program_byte_callback(u8 buff[]){
  /* Msg format : 4 bytes, address to program
   *              1 byte, data to program at address */
  u32 address = *(u32 *)&buff[0];
  u8 data = buff[4];

  /* TODO : Add check to restrict addresses that can be programmed? */

  /* Program specified address with data
   * Third argument is number of bytes to erase in parallel, shifted by 8
   *   to be OR'd in at the appropriate place in the FLASH CR register [9:8].
   *   Currently we just put in 0 as we only program 1 byte at a time.
   *   TODO : the interface for this could also be better in libopencm3,
   *   and could use some #defines
   */
  flash_unlock();
  flash_program_byte(address,data,0);
  flash_lock();

  /* Send message back to PC to signal operation is finished */
  debug_send_msg(MSG_STM_FLASH_COMPLETE,0,0);
}

void stm_flash_program_callback(u8 buff[]){
  /* Msg format : 4 bytes, address to program
   *              1 byte, number of addresses to program
   *              rest of bytes : data to program addr's with */
  u32 address = *(u32 *)&buff[0];
  u8 length = buff[4];
  u8 *data = &buff[5];

  /* TODO : Add check to restrict addresses that can be programmed? */

  /* Program specified addresses with data */
  flash_unlock();
  while (length) {
    /*program 1 byte at a time*/
    flash_program_byte(address,*data,0x00000000);
    length -= 1;
    data += 1;
    address += 1;
  }
  flash_lock();

  /* Send message back to PC to signal operation is finished */
  debug_send_msg(MSG_STM_FLASH_COMPLETE,0,0);
}

void stm_flash_read_callback(u8 buff[]){
  /*
   * Msg format : 4 bytes, starting address to read from
   *              1 byte, number of addresses to read
   */
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

  /* Send bytes to PC */
  debug_send_msg(MSG_STM_FLASH_READ,length+5,callback_data);
}

void stm_flash_callbacks_setup(){
  /* Create message callbacks node types to add to debug callback
   * linked list for each flash callback defined above */
  static msg_callbacks_node_t stm_flash_erase_sector_node, stm_flash_program_node, stm_flash_program_byte_node, stm_flash_read_node;
  /* Insert callbacks in debug callback linked list so they can be called */
  debug_register_callback(MSG_STM_FLASH_ERASE_SECTOR, &stm_flash_erase_sector_callback, &stm_flash_erase_sector_node);
  debug_register_callback(MSG_STM_FLASH_READ, &stm_flash_read_callback, &stm_flash_read_node);
  debug_register_callback(MSG_STM_FLASH_PROGRAM_BYTE, &stm_flash_program_byte_callback, &stm_flash_program_byte_node);
  debug_register_callback(MSG_STM_FLASH_PROGRAM, &stm_flash_program_callback, &stm_flash_program_node);
}
