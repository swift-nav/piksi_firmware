/*
 * Copyright (C) 2012 Henry Hallam <henry@swift-nav.com>
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

#ifndef SWIFTNAV_USART_H
#define SWIFTNAV_USART_H

#define USART_TX_BUFFER_LEN 4096
#define USART_RX_BUFFER_LEN 4096

#include <libopencm3/cm3/common.h>

void usart_setup_common(void);
void usart_tx_dma_setup(void);
void usart_rx_dma_setup(void);

u32 usart_write_dma(u8 data[], u32 n);

u32 usart_n_read_dma();
u32 usart_read_dma(u8 buff[], u32 len);

#endif /* SWIFTNAV_USART_H */

