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

#ifndef SWIFTNAV_DEBUG_MESSAGES_H
#define SWIFTNAV_DEBUG_MESSAGES_H

#include <libopencm3/cm3/common.h>

/* NOTE: This file is automatically parsed by a pyton script
 * to extract message definitions. Its therefore important to
 * stick to a certain format when declaring messages. You must
 * define the message with a line of the form:
 *
 * #define MSG_MSGNAME 0x?? // type
 *
 * Type may be either string or struct. Struct types must be
 * followed by a python struct format string in single-quotes.
 * (see: http://docs.python.org/library/struct.html)
 */

#define MSG_PRINT 0x01 // string
#define MSG_U32 0x03 // struct 'I'

#define MSG_POINT 0x02 // struct 'HHxxxxd'
typedef struct {
  u16 x;
  u16 y;
  double foo;
} point;


/* ----- Input Messages ----- */

#define MSG_FLASH_READ 0x80 // struct 'IB'
typedef struct {
  u32 address;
  u8 length;
} msg_flash_read_t;

#define MSG_FLASH_WRITE 0x81 // struct 'IB250B'
typedef struct {
  u32 address;
  u8 length;
  u8 data[250];
} msg_flash_write_t;

#define MSG_FLASH_ERASE_PAGE 0x82 // struct 'I'

#define MSG_FLASH_ERASE_ALL 0x83 // none

#endif
