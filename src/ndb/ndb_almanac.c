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

#define NDB_WEAK

#include <string.h>
#include <assert.h>
#include "ndb.h"
#include "ndb_internal.h"
#include <libswiftnav/constants.h>
#include "libswiftnav/logging.h"
#include "timing.h"
#include "sbp.h"
#include "sbp_utils.h"

#define NDB_ALMA_FILE_NAME   "alma"
static almanac_t ndb_almanac[PLATFORM_SIGNAL_COUNT] _CCM;
static ndb_element_metadata_t ndb_almanac_md[PLATFORM_SIGNAL_COUNT] _CCM;

static ndb_file_t ndb_alma_file = {
    .name = NDB_ALMA_FILE_NAME,
    .fh = -1,
    .expected_size =
          sizeof(almanac_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_file_end_mark),
    .data_size = sizeof(almanac_t),
    .n_elements = PLATFORM_SIGNAL_COUNT,
};

void ndb_almanac_init()
{
  ndb_load_data(&ndb_alma_file, "almanac", ndb_almanac, ndb_almanac_md,
                sizeof(almanac_t), PLATFORM_SIGNAL_COUNT);
}

enum ndb_op_code ndb_almanac_read(gnss_signal_t sid, almanac_t *a)
{
  u16 idx = sid_to_global_index(sid);
  ndb_retrieve(a, &ndb_almanac[idx], sizeof(almanac_t));

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_almanac_store(almanac_t *a, enum ndb_data_source src)
{
  u16 idx = sid_to_global_index(a->sid);
  return ndb_update(a, src, &ndb_almanac_md[idx]);
}

enum ndb_op_code ndb_almanac_cache_update(almanac_t *cached_a,
                                          ndb_update_counter_t *uc)
{
  enum ndb_op_code r = NDB_ERR_NONE;

  u16 idx = sid_to_global_index(cached_a->sid);
  ndb_element_metadata_t *md = &ndb_almanac_md[idx];

  ndb_lock();
  if (md->update_c != *uc) {
    r = ndb_almanac_read(cached_a->sid, cached_a);
    if (NDB_ERR_NONE != r) {
      ndb_unlock();
      return r;
    }
    *uc = md->update_c;
  }
  ndb_unlock();
  return r;
}

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_almanac_sbp_update()
{
}
