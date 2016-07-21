/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_INTERNAL_H
#define SWIFTNAV_TRACK_INTERNAL_H

#include <libswiftnav/common.h>
#include <libswiftnav/bit_sync.h>
#include <libswiftnav/signal.h>

#include "track_api.h"

/** \addtogroup tracking
 * \{ */

#define NAV_BIT_FIFO_SIZE 64 /**< Size of nav bit FIFO. Must be a power of 2 */

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE) - 1)
#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
          ((nav_bit_fifo_index_t)((write_index) - (read_index)))
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
          (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))

typedef struct {
  s8 soft_bit;
} nav_bit_fifo_element_t;

typedef u8 nav_bit_fifo_index_t;

typedef struct {
  nav_bit_fifo_index_t read_index;
  nav_bit_fifo_index_t write_index;
  nav_bit_fifo_element_t elements[NAV_BIT_FIFO_SIZE];
} nav_bit_fifo_t;

typedef struct {
  s32 TOW_ms;
  s8 bit_polarity;
  nav_bit_fifo_index_t read_index;
  bool valid;
} nav_time_sync_t;

typedef struct {
  /** FIFO for navigation message bits. */
  nav_bit_fifo_t nav_bit_fifo;
  /** Used to sync time decoded from navigation message
   * back to tracking channel. */
  nav_time_sync_t nav_time_sync;
  /** Time since last nav bit was appended to the nav bit FIFO. */
  u32 nav_bit_TOW_offset_ms;
  /** Bit sync state. */
  bit_sync_t bit_sync;
  /** Polarity of nav message bits. */
  s8 bit_polarity;
  /** Increments when tracking new signal. */
  u16 lock_counter;
  /** Set if this channel should output I/Q samples on SBP. */
  bool output_iq;
  /** Carrier phase integer offset in cycles. */
  double carrier_phase_offset;
} tracker_internal_data_t;

/** \} */

void track_internal_setup(void);

tracker_interface_list_element_t ** tracker_interface_list_ptr_get(void);

void tracker_internal_context_resolve(tracker_context_t *tracker_context,
                                      const tracker_channel_info_t **channel_info,
                                      tracker_internal_data_t **internal_data);

void internal_data_init(tracker_internal_data_t *internal_data,
                        gnss_signal_t sid);

void nav_bit_fifo_init(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_full(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                        const nav_bit_fifo_element_t *element);
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_fifo_element_t *element);
void nav_time_sync_init(nav_time_sync_t *sync);
bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                       s8 bit_polarity, nav_bit_fifo_index_t read_index);
bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                       s8 *bit_polarity, nav_bit_fifo_index_t *read_index);

s8 nav_bit_quantize(s32 bit_integrate);

u16 tracking_lock_counter_increment(gnss_signal_t sid);
u16 tracking_lock_counter_get(gnss_signal_t sid);

#endif
