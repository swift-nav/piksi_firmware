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

#ifndef _SWIFTNAV_3DRRADIO_H_
#define _SWIFTNAV_3DRRADIO_H_

#include <libswiftnav/common.h>
#include "usart.h"
#include "usart_chat.h"

void radio_preconfigure_hook(enum uart u, u32 default_baud, char* uart_name);
void radio_setup();

#endif  /* _SWIFTNAV_3DRRADIO_H_ */
