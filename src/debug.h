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

#ifndef SWIFTNAV_DEBUG_H
#define SWIFTNAV_DEBUG_H

#include "debug_messages.h"

#define DEBUG_MAGIC_1 0xBE
#define DEBUG_MAGIC_2 0xEF

#define DEBUG_MSG(msg_type, item) \
  send_debug_msg(msg_type, sizeof(item), (u8*)&(item))
void debug_setup();
void send_debug_msg(u8 msg_type, u8 len, u8 buff[]);
//u8 get_debug_msg(u8 *type, u8 *len, u8 *buff[]);


#endif
