/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SETTINGS_H
#define SWIFTNAV_SETTINGS_H

#include <libswiftnav/common.h>

/** \addtogroup io
 * \{ */

/** Message and baud rate settings for a USART. */
typedef struct {
  enum {
    SBP,
    NMEA,
    RTCM
  } mode; /** Communication mode : Swift Binary Protocol or NMEA */
  u32 baud_rate;
  u16 message_mask;
} usart_settings_t;

/** Message and baud rate settings for all USARTs. */
typedef struct {
  enum {
    VALID = 0,
    /** Settings area is erased, i.e. all 0xFF. */
    INVALID = 0xFF
  } settings_valid;
  usart_settings_t ftdi_usart;
  usart_settings_t uarta_usart;
  usart_settings_t uartb_usart;
} settings_t;

/** \} */

extern settings_t settings;

#endif  /* SWIFTNAV_SETTINGS_H */

