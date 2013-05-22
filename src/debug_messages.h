/*
 * Copyright (C) 2012 Fergus Noble <fergusnoble@gmail.com>
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

#ifndef SWIFTNAV_DEBUG_MESSAGES_H
#define SWIFTNAV_DEBUG_MESSAGES_H

#define MSG_PRINT 0x01

#define MSG_ACQ_SETUP 0x69

#define MSG_BOOTLOADER_HANDSHAKE   0xB0 /* Callback in C and Python */
#define MSG_BOOTLOADER_JUMP_TO_APP 0xB1 /* Callback in C */

#define MSG_CW_START   0xC1
#define MSG_CW_RESULTS 0xC0

#define MSG_NAP_DEVICE_DNA 0xDD /* Callback in C and Python */

#define MSG_STM_FLASH_WRITE 0xE0 /* Callback in C */
#define MSG_STM_FLASH_READ  0xE1 /* Callback in C and Python */
#define MSG_STM_FLASH_ERASE 0xE2 /* Callback in C */
#define MSG_STM_FLASH_DONE  0xE0 /* Callback in Python */

#define MSG_STM_UNIQUE_ID 0xE5

#define MSG_M25_FLASH_WRITE 0xF0 /* Callback in C */
#define MSG_M25_FLASH_READ  0xF1 /* Callback in C and Python */
#define MSG_M25_FLASH_ERASE 0xF2 /* Callback in C */
#define MSG_M25_FLASH_DONE  0xF0 /* Callback in Python */

#define MSG_SOLUTION 0x50
#define MSG_DOPS     0x51
#define MSG_PR_ERRS  0x52

#define MSG_TRACKING_STATE 0x22

#endif /* SWIFTNAV_DEBUG_MESSAGES_H */
