/*
 * Copyright (C) 2012 Henry Hallam <henry@swift-nav.com>
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

#ifndef SWIFTNAV_USART_H
#define SWIFTNAV_USART_H

#define USART_TX_BUFFER_LEN 4096
#define USART_RX_BUFFER_LEN 4096

#include <libopencm3/cm3/common.h>

typedef struct {
  u8 buff[USART_RX_BUFFER_LEN];
  u32 rd;
  u32 rd_wraps;
  u32 wr_wraps;

  u32 dma;
  u32 usart;
  u8 stream;
  u8 channel;
} usart_rx_dma_state;

typedef struct {
  u8 buff[USART_TX_BUFFER_LEN];
  u32 rd;
  u32 wr;
  u32 xfer_len;
  u32 feif_isrs;

  u32 dma;
  u32 usart;
  u8 stream;
  u8 channel;
} usart_tx_dma_state;

extern const u8 dma_irq_lookup[2][8];

void usart_setup_common(void);

void usart_tx_dma_setup(usart_tx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
u32 usart_tx_n_free(usart_tx_dma_state* s);
void usart_tx_dma_isr(usart_tx_dma_state* s);
u32 usart_write_dma(usart_tx_dma_state* s, u8 data[], u32 len);

void usart_rx_dma_setup(usart_rx_dma_state* s, u32 usart,
                        u32 dma, u8 stream, u8 channel);
void usart_rx_dma_disable(usart_rx_dma_state* s);
void usart_rx_dma_isr(usart_rx_dma_state* s);
u32 usart_n_read_dma(usart_rx_dma_state* s);
u32 usart_read_dma(usart_rx_dma_state* s, u8 data[], u32 len);

#endif /* SWIFTNAV_USART_H */

