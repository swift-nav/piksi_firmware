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

#include <libopencm3/cm3/common.h>

typedef struct {
  enum {
    PIKSI_BINARY,
    NMEA
  } mode;
  u32 baud_rate;
  u8 message_mask;
} usart_settings_t;

typedef struct {
  usart_settings_t ftdi_usart;
  usart_settings_t uarta_usart;
  usart_settings_t uartb_usart;
} settings_t;

extern settings_t settings;


