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

#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"

/** \addtogroup io
 * \{ */

settings_t settings /*__attribute__ ((section(".settings_area"))) */=
/* Default settings: */
{
  .settings_valid = VALID,

  .ftdi_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_FTDI,
    .message_mask = 0xFFFF,
  },
  .uarta_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_TTL,
    .message_mask = 0x40,
  },
  .uartb_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_TTL,
    .message_mask = 0xFF00
  },
};

/** \} */

