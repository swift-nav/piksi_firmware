/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "peripherals/usart.h"

void usart_support_init(void)
{
}

void usart_support_set_parameters(void *sd, u32 baud)
{
  if (sd == NULL)
    return;

  SerialConfig config = {
    .speed = baud,
  };
  sdStop(sd);

  /* NOTE This depends on the SerialDriver not keeping the config struct. */
  sdStart(sd, &config);
}

void usart_support_disable(void *sd)
{
  if (sd == NULL)
    return;

  sdStop(sd);
}

u32 usart_support_n_read(void *sd)
{
  chSysLock();
  u32 n = chQSpaceI(&((SerialDriver*)sd)->iqueue);
  chSysUnlock();
  return n;
}

u32 usart_support_tx_n_free(void *sd)
{
  chSysLock();
  u32 n = chQSpaceI(&((SerialDriver*)sd)->oqueue);
  chSysUnlock();
  return n;
}

u32 usart_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout)
{
  return chnReadTimeout((SerialDriver*)sd, data, len, timeout);
}

u32 usart_support_write(void *sd, const u8 data[], u32 len)
{
  return chnWriteTimeout((SerialDriver*)sd, data, len, TIME_IMMEDIATE);
}

