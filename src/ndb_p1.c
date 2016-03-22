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
#include <string.h>
#include "ndb.h"
#include "libswiftnav/logging.h"
#include "timing.h"
#include "sbp.h"
#include "sbp_utils.h"

ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT] _CCM;
ndb_element_metadata_t ndb_ephemeris_md[PLATFORM_SIGNAL_COUNT] _CCM;
almanac_t ndb_almanac[PLATFORM_SIGNAL_COUNT] _CCM;
ndb_element_metadata_t ndb_almanac_md[PLATFORM_SIGNAL_COUNT] _CCM;

#define NDB_EPHE_FILE_NAME   "ephe"
#define NDB_ALMA_FILE_NAME   "alma"

ndb_file_t ndb_ephe_file = { .name = NDB_EPHE_FILE_NAME, .fh = -1,
    .expected_size = sizeof(ndb_file_version)
        + sizeof(ephemeris_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT };

ndb_file_t ndb_alma_file = { .name = NDB_ALMA_FILE_NAME, .fh = -1,
    .expected_size = sizeof(ndb_file_version)
        + sizeof(almanac_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT };

enum ndb_op_code ndb_p1_init()
{
  u32 i;

  memset(ndb_ephemeris, 0, sizeof(ndb_ephemeris));
  for (u32 i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_ephemeris[i].sid = sid_from_global_index(i);
  }

  if (ndb_load_data(&ndb_ephe_file, ndb_ephemeris, ndb_ephemeris_md,
                    sizeof(ephemeris_t), PLATFORM_SIGNAL_COUNT)
      != NDB_ERR_NONE) {
    log_info("No ephemeris file present in flash, create an empty one");
  } else
    log_info("Ephemerides loaded from flash");

  for (i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_ephemeris_md[i].data = &ndb_ephemeris[i];
    ndb_ephemeris_md[i].data_size = sizeof(ephemeris_t);
    ndb_ephemeris_md[i].index = i;
    ndb_ephemeris_md[i].n_elements = PLATFORM_SIGNAL_COUNT;
    ndb_ephemeris_md[i].file = &ndb_ephe_file;
    ndb_ephemeris_md[i].next = NULL;
    ndb_ephemeris_md[i].update_c = 1;
  }

  memset(ndb_almanac, 0, sizeof(ndb_almanac));
  for (u32 i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_almanac[i].sid = sid_from_global_index(i);
  }

  if (ndb_load_data(&ndb_alma_file, ndb_almanac, ndb_almanac_md,
                    sizeof(almanac_t), PLATFORM_SIGNAL_COUNT) != NDB_ERR_NONE) {
    log_info("No almanac file present in flash, create an empty one");
  } else
    log_info("Almanacs loaded from flash");

  for (i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_almanac_md[i].data = &ndb_almanac[i];
    ndb_almanac_md[i].data_size = sizeof(almanac_t);
    ndb_almanac_md[i].index = i;
    ndb_almanac_md[i].n_elements = PLATFORM_SIGNAL_COUNT;
    ndb_almanac_md[i].file = &ndb_alma_file;
    ndb_almanac_md[i].next = NULL;
    ndb_almanac_md[i].update_c = 1;
  }
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx = sid_to_global_index(sid);
  ndb_retrieve(e, &ndb_ephemeris[idx], sizeof(ephemeris_t));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_update_cache_ephemeris(ephemeris_t *cached_e,
                                            ndb_update_counter_t *uc)
{
  u16 idx = sid_to_global_index(cached_e->sid);
  ndb_element_metadata_t *md = &ndb_ephemeris_md[idx];

  if (md->update_c == *uc) {
    return NDB_ERR_NONE;
  }

  enum ndb_op_code r = ndb_ephemeris_read(cached_e->sid, cached_e);
  if (NDB_ERR_NONE != r) {
    return r;
  }

  *uc = md->update_c;
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_ephemeris_store(ephemeris_t *e, enum ndb_data_source src)
{
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }
  u16 idx = sid_to_global_index(e->sid);
  return ndb_update(&ndb_ephemeris[idx], e, sizeof(ephemeris_t), src,
                    &ndb_ephemeris_md[idx], 0xff, NDB_IE_VALID);
}

enum ndb_op_code ndb_ephemeris_info(gnss_signal_t sid, u8* v, u8* h,
                                    gps_time_t* toe, u8* fit_interval)
{
  u16 idx = sid_to_global_index(sid);
  ndb_lock(1);
  *v = ndb_ephemeris[idx].valid;
  *h = ndb_ephemeris[idx].healthy;
  if (NULL != toe) {
    *toe = ndb_ephemeris[idx].toe;
  }
  if (NULL != fit_interval) {
    *fit_interval = ndb_ephemeris[idx].fit_interval;
  }
  ndb_lock(0);
  return NDB_ERR_NONE;
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
  return ndb_update(&ndb_almanac[idx], a, sizeof(almanac_t), src,
                    &ndb_almanac_md[idx], 0xff, NDB_IE_VALID);
}

enum ndb_op_code ndb_update_cache_almanac(almanac_t *cached_a,
                                          ndb_update_counter_t *uc)
{
  u16 idx = sid_to_global_index(cached_a->sid);
  ndb_element_metadata_t *md = &ndb_almanac_md[idx];

  if (md->update_c == *uc) {
    return NDB_ERR_NONE;
  }

  enum ndb_op_code r = ndb_almanac_read(cached_a->sid, cached_a);
  if (NDB_ERR_NONE != r) {
    return r;
  }

  *uc = md->update_c;
  return NDB_ERR_NONE;
}

#define EPHEMERIS_MESSAGE_SPACING_cycle        (200 / NV_WRITE_REQ_TIMEOUT)
#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle (15 * 1000 / NV_WRITE_REQ_TIMEOUT)

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_p1_sbp_update()
{
  static u32 count = 0;
  static u32 i = 0;
  static bool tx_en = true; /* initially enable SBP TX */

  if (tx_en) {
    if (!(count % EPHEMERIS_MESSAGE_SPACING_cycle)) {
      /* every 200 ms send eph of a SV */
      ephemeris_t e;
      gps_time_t t = get_current_time();
      gnss_signal_t sid = sid_from_global_index(i);
      ndb_ephemeris_read(sid, &e);
      if (ephemeris_valid(&e, &t)) {
        msg_ephemeris_t msg;
        pack_ephemeris(&e, &msg);
        sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *) &msg);
      }
      i++;
      if (i == PLATFORM_SIGNAL_COUNT) {
        /* no eph to send */
        i = 0;
        tx_en = false;
      }
    }
  } else {
    if (!(count % EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle)) {
      /* every 15 sec enable tx again */
      count = 0;
      tx_en = true;
      return;
    }
  }

  count++;
}
