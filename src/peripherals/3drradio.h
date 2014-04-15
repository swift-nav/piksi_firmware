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

bool busy_wait_for_ok(u32 usart, u32 ms);
u32 radio_preconfigure_hook(u32 usart);

#endif  /* _SWIFTNAV_3DRRADIO_H_ */