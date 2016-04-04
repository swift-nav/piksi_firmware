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

#include "track_internal.h"
#include "track.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <ch.h>

#include "signal.h"
#include "peripherals/random.h"

/** \addtogroup tracking
 * \{ */

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

static tracker_interface_list_element_t *tracker_interface_list = 0;

/* signal lock counter
 * A map of signal to an initially random number that increments each time that
 * signal begins being tracked.
 */
static u16 tracking_lock_counters[PLATFORM_SIGNAL_COUNT];

/** Set up internal tracker data. */
void track_internal_setup(void)
{
  for (u32 i=0; i < PLATFORM_SIGNAL_COUNT; i++) {
    tracking_lock_counters[i] = rand();
  }
}

/** Return a pointer to the tracker interface list. */
tracker_interface_list_element_t ** tracker_interface_list_ptr_get(void)
{
  return &tracker_interface_list;
}

/** Initialize a tracker internal data structure.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param sid               Signal identifier to use.
 */
void internal_data_init(tracker_internal_data_t *internal_data,
                        gnss_signal_t sid)
{
  /* Initialize all fields to 0 */
  memset(internal_data, 0, sizeof(tracker_internal_data_t));

  internal_data->bit_polarity = BIT_POLARITY_UNKNOWN;

  nav_bit_fifo_init(&internal_data->nav_bit_fifo);
  nav_time_sync_init(&internal_data->nav_time_sync);
  bit_sync_init(&internal_data->bit_sync, sid);
}

/** Initialize a nav_bit_fifo_t struct.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 */
void nav_bit_fifo_init(nav_bit_fifo_t *fifo)
{
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Determine if a nav bit FIFO is full.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 *
 * \return true if the nav bit FIFO is full, false otherwise.
 */
bool nav_bit_fifo_full(nav_bit_fifo_t *fifo)
{
  return (NAV_BIT_FIFO_LENGTH(fifo) == NAV_BIT_FIFO_SIZE);
}

/** Write data to the nav bit FIFO.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                        const nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) < NAV_BIT_FIFO_SIZE) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(&fifo->elements[fifo->write_index & NAV_BIT_FIFO_INDEX_MASK],
           element, sizeof(nav_bit_fifo_element_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->write_index++;
    return true;
  }

  return false;
}

/** Read pending data from the nav bit FIFO.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) > 0) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(element, &fifo->elements[fifo->read_index & NAV_BIT_FIFO_INDEX_MASK],
           sizeof(nav_bit_fifo_element_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** Initialize a nav_time_sync_t struct.
 *
 * \param sync          nav_time_sync_t struct to use.
 */
void nav_time_sync_init(nav_time_sync_t *sync)
{
  sync->valid = false;
}

/** Write pending time sync data from the decoder thread.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if data was stored successfully, false otherwise.
 */
bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                       s8 bit_polarity, nav_bit_fifo_index_t read_index)
{
  bool result = false;

  if (!sync->valid) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    sync->TOW_ms = TOW_ms;
    sync->bit_polarity = bit_polarity;
    sync->read_index = read_index;
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    sync->valid = true;
    result = true;
  }

  return result;
}

/** Read pending time sync data provided by the decoder thread.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if outputs are valid, false otherwise.
 */
bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                       s8 *bit_polarity, nav_bit_fifo_index_t *read_index)
{
  bool result = false;

  if (sync->valid) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    *TOW_ms = sync->TOW_ms;
    *bit_polarity = sync->bit_polarity;
    *read_index = sync->read_index;
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    sync->valid = false;
    result = true;
  }

  return result;
}

/** Compress a 32 bit integration value down to 8 bits.
 *
 * \param bit_integrate   Signed bit integration value.
 */
s8 nav_bit_quantize(s32 bit_integrate)
{
  //  0 through  2^24 - 1 ->  0 = weakest positive bit
  // -1 through -2^24     -> -1 = weakest negative bit

  if (bit_integrate >= 0)
    return bit_integrate / (1 << 24);
  else
    return ((bit_integrate + 1) / (1 << 24)) - 1;
}

/** Increment and return the tracking lock counter for the specified sid.
 *
 * \param sid         Signal identifier to use.
 */
u16 tracking_lock_counter_increment(gnss_signal_t sid)
{
  return ++tracking_lock_counters[sid_to_global_index(sid)];
}

/** Return the tracking lock counter for the specified sid.
 *
 * \param sid         Signal identifier to use.
 */
u16 tracking_lock_counter_get(gnss_signal_t sid)
{
  return tracking_lock_counters[sid_to_global_index(sid)];
}

/** \} */
