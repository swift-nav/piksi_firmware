/*
 * Copyright (C) 2012-2013 Swift Navigation Inc.
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

#include "settings.h"
#include "peripherals/usart.h"

/** \addtogroup io
 * \{ */

settings_t settings __attribute__ ((section (".settings_area"))) =
/* Default settings: */
{
  .settings_valid = VALID,

  .ftdi_usart = {
    .mode = SBP,
    .baud_rate = USART_DEFAULT_BAUD,
    .message_mask = 0xFF,
  },
  .uarta_usart = {
    .mode = SBP,
    .baud_rate = USART_DEFAULT_BAUD,
    .message_mask = 0xFF,
  },
  .uartb_usart = {
    .mode = NMEA,
    .baud_rate = 115200,
  },
};

/** \} */

#endif /* SWIFTNAV_SETTINGS_H */
