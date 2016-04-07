/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "signal.h"

#include <assert.h>
#include <string.h>

/** \defgroup signal GNSS signal identifiers (SID)
 * \{ */

/** Table of global and constellation start indexes, monotonically increasing,
 * indexed by code. */
typedef struct {
  u16 constellation_start_index;
  u16 global_start_index;
} code_table_element_t;
static code_table_element_t code_table[CODE_COUNT];

/** Number of signals for each code which are supported on
 * the current hardware platform. */
static const u16 code_signal_counts[CODE_COUNT] = {
  [CODE_GPS_L1CA] = PLATFORM_SIGNAL_COUNT_GPS_L1CA,
  [CODE_GPS_L2CM] = PLATFORM_SIGNAL_COUNT_GPS_L2CM,
  [CODE_SBAS_L1CA] = PLATFORM_SIGNAL_COUNT_SBAS_L1CA,
  [CODE_GLO_L1CA] = PLATFORM_SIGNAL_COUNT_GLO_L1CA,
  [CODE_GLO_L2CA] = PLATFORM_SIGNAL_COUNT_GLO_L2CA,
};

/** Initialize the signal module. */
void signal_init(void)
{
  /* Populate constellation start index */
  u16 constellation_start_indexes[CONSTELLATION_COUNT];
  memset(constellation_start_indexes, 0, sizeof(constellation_start_indexes));
  for (enum code code = 0; code < CODE_COUNT; code++) {
    enum constellation constellation = code_to_constellation(code);
    code_table[code].constellation_start_index =
        constellation_start_indexes[constellation];
    constellation_start_indexes[constellation] += code_signal_counts[code];
  }

  /* Populate global start index */
  u16 global_start_index = 0;
  for (enum code code = 0; code < CODE_COUNT; code++) {
    code_table[code].global_start_index = global_start_index;
    global_start_index += code_signal_counts[code];
  }
}

/** Convert a global signal index to a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param global_index    Global signal index in [0, PLATFORM_SIGNAL_COUNT).
 *
 * \return gnss_signal_t corresponding to global_index.
 */
gnss_signal_t sid_from_global_index(u16 global_index)
{
  for (enum code code = 0; code < CODE_COUNT; code++) {
    if (global_index < code_table[code].global_start_index +
        code_signal_counts[code]) {
      return sid_from_code_index(code, global_index -
          code_table[code].global_start_index);
    }
  }

  assert(!"Invalid global index");
  return construct_sid(CODE_INVALID, 0);
}

/** Convert a constellation-specific signal index to a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param constellation         Constellation to use.
 * \param constellation_index   Constellation-specific signal index in
 *                              [0, PLATFORM_SIGNAL_COUNT_\<constellation\>).
 *
 * \return gnss_signal_t corresponding to constellation and constellation_index.
 */
gnss_signal_t sid_from_constellation_index(enum constellation constellation,
                                           u16 constellation_index)
{
  for (enum code code = 0; code < CODE_COUNT; code++) {
    if (code_to_constellation(code) == constellation) {
      if (constellation_index < code_table[code].constellation_start_index +
          code_signal_counts[code]) {
        return sid_from_code_index(code, constellation_index -
            code_table[code].constellation_start_index);
      }
    }
  }

  assert(!"Invalid constellation index");
  return construct_sid(CODE_INVALID, 0);
}

/** Return the global signal index for a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return Global signal index in [0, PLATFORM_SIGNAL_COUNT).
 */
u16 sid_to_global_index(gnss_signal_t sid)
{
  assert(code_supported(sid.code));
  return code_table[sid.code].global_start_index +
      sid_to_code_index(sid);
}

/** Return the constellation-specific signal index for a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return Constellation-specific signal index in
 *         [0, PLATFORM_SIGNAL_COUNT_\<constellation\>).
 */
u16 sid_to_constellation_index(gnss_signal_t sid)
{
  assert(code_supported(sid.code));
  return code_table[sid.code].constellation_start_index +
      sid_to_code_index(sid);
}

/** Determine if a gnss_signal_t is valid and supported on the current
 * hardware platform.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return true if sid is valid and supported, false otherwise.
 */
bool sid_supported(gnss_signal_t sid)
{
  /* Verify general validity */
  if (!sid_valid(sid))
    return false;

  /* Verify that the code is supported on this platform */
  if (!code_supported(sid.code))
    return false;

  return true;
}

/** Determine if a code is valid and supported on the current
 * hardware platform.
 *
 * \param code  Code to use.
 *
 * \return true if code is valid and supported, false otherwise.
 */
bool code_supported(enum code code)
{
  /* Verify general validity */
  if (!code_valid(code))
    return false;

  /* Verify that the code is supported on this platform */
  if (code_signal_counts[code] == 0)
    return false;

  return true;
}

/* \} */
