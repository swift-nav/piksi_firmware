/*
 * Copyright (C) 2012-2015 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _SWIFTNAV_OPENLOG_H_
#define _SWIFTNAV_OPENLOG_H_

#include <libswiftnav/common.h>
#include "usart.h"

void openlog_setup();
void openlog_configure_hook(usart_dma_state *s, char* uart_name);
void openlog_file_break();

#endif  /* _SWIFTNAV_OPENLOG_H_ */
