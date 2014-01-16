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

#include <string.h>

#include "error.h"
#include "sbp.h"
#include "peripherals/stm_flash.h"
#include "board/m25_flash.h"
#include "flash_callbacks.h"
#include "main.h"

/** Callback to erase a sector of either the STM or M25 flash.
 *
 * \param buff Array of u8 (length 2) :
 *             - [0] Flash to program (see flash_callbacks.h for #defines)
 *             - [1] Flash sector number to erase (0-11)
 */
void flash_erase_sector_callback(u8 buff[])
{
  u8 flash = buff[0];
  u8 sector = buff[1];

  if (flash == FLASH_STM) {
    stm_flash_erase_sector(sector);
  } else if (flash == FLASH_M25) {
    u32 addr = ((u32)sector) << 16;
    m25_write_enable();
    m25_sector_erase(addr);
    m25_write_disable();
  } else {
    screaming_death("flash_erase_callback passed invalid flash argument");
  }

  /* Keep trying to send message until it makes it into the buffer. */
  while (sbp_send_msg(MSG_FLASH_DONE, 0, 0)) ;
}

/** Callback to program a set of addresses of either the STM or M25 flash.
 * Note : sector containing addresses must be erased before addresses can be
 * programmed.
 *
 * \param buff Array of u8 (length >= 7) :
 *             - [0]     Flash to program (FLASH_STM or FLASH_M25)
 *             - [1:4]   Starting address of set to program
 *             - [5]     Length of set of addresses to program - counts up
 *                       from starting address
 *             - [6:end] Data to program addresses with
 */
void flash_program_callback(u8 buff[])
{
  u8 flash = buff[0];
  u32 address = *(u32 *)&buff[1];
  u8 length = buff[5];
  u8 *data = &buff[6];

  if (length > FLASH_ADDRS_PER_OP)
    screaming_death("flash_program_callback received length > " STR(FLASH_ADDRS_PER_OP));

  if (flash == FLASH_STM) {
    stm_flash_program(address, data, length);
  } else if (flash == FLASH_M25) {
    m25_write_enable();
    m25_page_program(address, data, length);
    m25_write_disable();
  } else {
    screaming_death("flash_program_callback passed invalid flash argument");
  }

  /* Keep trying to send message until it makes it into the buffer. */
  while (sbp_send_msg(MSG_FLASH_DONE, 0, 0)) ;
}

/** Callback to read a set of addresses of either the STM or M25 flash.
 *
 * \param buff Array of u8 (length 5) :
 *             - [0]   Flash to read (FLASH_STM or FLASH_M25)
 *             - [1:4] Starting address of set to read
 *             - [5]   Length of set of addresses to read - counts up from
 *                     starting address
 */
void flash_read_callback(u8 buff[])
{
  u8 flash = buff[0];
  u32 address = *(u32 *)&buff[1];
  u8 length = buff[5];

  /* TODO: figure out what max length is, make it a macro */
  if (length > FLASH_ADDRS_PER_OP)
    screaming_death("flash_read_callback received length > " STR(FLASH_ADDRS_PER_OP));
  if (flash == FLASH_STM) {
    if (address < STM_FLASH_MIN_ADDR)
      screaming_death("flash_read_callback received address < " STR(STM_FLASH_MIN_ADDR));
    if (address+length-1 > STM_FLASH_MAX_ADDR)
      screaming_death("flash_read_callback received address+length-1 > " STR(STM_FLASH_MAX_ADDR));
  }

  u8 callback_data[length + 5];
  callback_data[0] = (address >> 0) & 0xFF;
  callback_data[1] = (address >> 8) & 0xFF;
  callback_data[2] = (address >> 16) & 0xFF;
  callback_data[3] = (address >> 24) & 0xFF;
  callback_data[4] = length;

  if (flash == FLASH_STM) {
    memcpy(&callback_data[5], (const void *)address, length);
  } else if (flash == FLASH_M25) {
    m25_read(address, &callback_data[5], length);
  } else {
    screaming_death("flash_read_callback passed invalid flash argument");
  }

  while (sbp_send_msg(MSG_FLASH_READ, length + 5, callback_data)) ;
}

/** Callback to write to the 8-bit M25 flash status register.
 *
 * \param buff Array of u8 (length == 1) :
 *             - [0] Byte to write to the M25 flash status register.
 */
void m25_flash_write_status_callback(u8 buff[])
{
  u8 sr = buff[0];
  m25_write_enable();
  m25_write_status(sr);
  m25_write_disable();
  while (sbp_send_msg(MSG_FLASH_DONE, 0, 0)) ;
}

/** Callback to unlock a sector of the STM flash memory.
 *
 * \param buff Array of u8 (length 1) :
 *             - [0] flash sector number to unlock.
 */
void stm_flash_unlock_sector_callback(u8 buff[])
{
  u8 sector = buff[0];
  stm_flash_unlock_sector(sector);
  while (sbp_send_msg(MSG_FLASH_DONE, 0, 0)) ;
}

/** Callback to lock a sector of the STM flash memory.
 *
 * \param buff Array of u8 (length 1) :
 *             - [0] Flash sector number to unlock.
 */
void stm_flash_lock_sector_callback(u8 buff[])
{
  u8 sector = buff[0];
  stm_flash_lock_sector(sector);
  while (sbp_send_msg(MSG_FLASH_DONE, 0, 0)) ;
}

/** Set up flash callbacks. */
void flash_callbacks_register(void)
{
  static msg_callbacks_node_t flash_erase_sector_node;
  static msg_callbacks_node_t flash_read_node;
  static msg_callbacks_node_t flash_program_node;

  static msg_callbacks_node_t stm_flash_lock_sector_node;
  static msg_callbacks_node_t stm_flash_unlock_sector_node;

  static msg_callbacks_node_t m25_flash_write_status_node;

  sbp_register_callback(MSG_FLASH_ERASE,
                        &flash_erase_sector_callback,
                        &flash_erase_sector_node);
  sbp_register_callback(MSG_FLASH_READ,
                        &flash_read_callback,
                        &flash_read_node);
  sbp_register_callback(MSG_FLASH_PROGRAM,
                        &flash_program_callback,
                        &flash_program_node);

  sbp_register_callback(MSG_STM_FLASH_LOCK_SECTOR,
                        &stm_flash_lock_sector_callback,
                        &stm_flash_lock_sector_node);
  sbp_register_callback(MSG_STM_FLASH_UNLOCK_SECTOR,
                        &stm_flash_unlock_sector_callback,
                        &stm_flash_unlock_sector_node);

  sbp_register_callback(MSG_M25_FLASH_WRITE_STATUS,
                        &m25_flash_write_status_callback,
                        &m25_flash_write_status_node);
}

/** Callback to read STM32F4's hardcoded unique ID.
 * Sends STM32F4 unique ID (12 bytes) back to host.
 */
void stm_unique_id_callback(u8 buff[] __attribute__((unused)))
{
  while (sbp_send_msg(MSG_STM_UNIQUE_ID, 12, (u8*)STM_UNIQUE_ID_ADDR)) ;
}

/** Register callback to read Device's Unique ID. */
void stm_unique_id_callback_register(void)
{
  static msg_callbacks_node_t stm_unique_id_node;

  sbp_register_callback(MSG_STM_UNIQUE_ID,
                        &stm_unique_id_callback,
                        &stm_unique_id_node);
}

