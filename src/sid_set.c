/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>
#include <libswiftnav/bits.h>
#include "sid_set.h"

/** Initialize new sid set
 *
 * \param sid_list gnss_signal_list_t to be initialized
 *
 */
void sid_set_init(gnss_signal_set_t sid_list)
{
  memset(sid_list, 0, sizeof(gnss_signal_set_t));
}

/** Add new element to the list
 *
 * \param sid_list gnss_signal_list_t to add new element to
 * \param sid element to be added
 *
 */
void sid_set_add(gnss_signal_set_t sid_list, const gnss_signal_t sid)
{
  u32 m = 0x01 << sid_to_code_index(sid);
  sid_list[sid.code] |= m;
}

/** Get number of satellites in sid list
 *
 * \param sid_list gnss_signal_list_t to add new element to
 *
 * \returns Number of unique satellites present in the list.
 *
 */
u32 sid_set_get_sat_count(const gnss_signal_set_t sid_list)
{
  u32 cnt = get_bits_number(sid_list[CODE_GPS_L1CA] |
                            sid_list[CODE_GPS_L2CM], 1);

  cnt += get_bits_number(sid_list[CODE_SBAS_L1CA], 1);

  cnt += get_bits_number(sid_list[CODE_GLO_L1CA] |
                         sid_list[CODE_GLO_L2CA], 1);
  return cnt;
}
