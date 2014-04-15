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


#include "3drradio.h"
#include <libopencm3/stm32/f4/usart.h>

void radio_preconfigure_hook(u32 usart)
{
	printf("FUCK YOUUUU\n");
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);
	usart_send_blocking(usart, 0xDD);

}
