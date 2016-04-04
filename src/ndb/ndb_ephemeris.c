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
#define DEBUG 1

#include <string.h>
#include <assert.h>
#include "ndb.h"
#include "ndb_internal.h"
#include <libswiftnav/constants.h>
#include "libswiftnav/logging.h"
#include "timing.h"
#include "sbp.h"
#include "sbp_utils.h"

#define NDB_EPHE_FILE_NAME   "ephe"

static ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT] _CCM;
static ndb_element_metadata_t ndb_ephemeris_md[PLATFORM_SIGNAL_COUNT];
static ndb_file_t ndb_ephe_file = {
    .name = NDB_EPHE_FILE_NAME,
    .fh = -1,
    .expected_size =
          sizeof(ephemeris_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_file_end_mark),
    .data_size = sizeof(ephemeris_t),
    .n_elements = PLATFORM_SIGNAL_COUNT,
};

typedef struct {
  ephemeris_t ephe;
  bool used;
  ndb_timestamp_t received_at;
} ephemeris_candidate_t;

typedef enum {
  EPHE_IDENTICAL,
  EPHE_NEW_CANDIDATE,
  EPHE_NEW_TRUSTED,
  EPHE_CAND_MISMATCH,
} ndb_ephemeris_status_t;

#define EPHE_CAND_LIST_LEN (MAX_CHANNELS)
#define MAX_EPHE_CANDIDATE_AGE 92 /* seconds */
static ephemeris_candidate_t ephe_candidates[EPHE_CAND_LIST_LEN] _CCM;
static MUTEX_DECL(cand_list_access);

#define EPHEMERIS_MESSAGE_SPACING_cycle        (200 / NV_WRITE_REQ_TIMEOUT)
#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle (15000 / NV_WRITE_REQ_TIMEOUT)

void ndb_ephemeris_init()
{
  memset(ephe_candidates, 0, sizeof(ephe_candidates));

  ndb_load_data(&ndb_ephe_file, "ephemeris", ndb_ephemeris, ndb_ephemeris_md,
                sizeof(ephemeris_t), PLATFORM_SIGNAL_COUNT);
}

enum ndb_op_code ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx = sid_to_global_index(sid);
  ndb_retrieve(e, &ndb_ephemeris[idx], sizeof(ephemeris_t));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_ephemeris_cache_update(ephemeris_t *cached_e,
                                            ndb_update_counter_t *uc)
{
  enum ndb_op_code r = NDB_ERR_NONE;

  u16 idx = sid_to_global_index(cached_e->sid);
  ndb_element_metadata_t *md = &ndb_ephemeris_md[idx];

  ndb_lock();
  if (md->update_c != *uc) {
    r = ndb_ephemeris_read(cached_e->sid, cached_e);
    if (NDB_ERR_NONE != r) {
      ndb_unlock();
      return r;
    }
    *uc = md->update_c;
  }
  ndb_unlock();
  return r;
}

s16 ndb_ephe_find_candidate(const ephemeris_t *new)
{
  int i;
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    if (ephe_candidates[i].used &&
        sid_is_equal(ephe_candidates[i].ephe.sid, new->sid))
      return i;
  }
  return -1;
}

void ndb_ephe_try_adding_candidate(const ephemeris_t *new)
{
  int i;
  u32 candidate_age;
  ndb_timestamp_t now  = ndb_get_timestamp();
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    bool empty = true;
    if(ephe_candidates[i].used) {
      candidate_age = ST2S(now - ephe_candidates[i].received_at);
      empty = candidate_age >  MAX_EPHE_CANDIDATE_AGE;
    }

    if (empty) {
      memcpy(&ephe_candidates[i].ephe, new, sizeof(ephemeris_t));
      ephe_candidates[i].received_at = ndb_get_timestamp();
      ephe_candidates[i].used = true;
      return;
    }
  }
}

void ndb_ephe_release_candidate(s16 cand_index)
{
  if((cand_index < 0) || (cand_index >= EPHE_CAND_LIST_LEN))
    return;
  ephe_candidates[cand_index].used = false;
}

ndb_ephemeris_status_t ndb_get_ephemeris_status(const ephemeris_t *new)
{
  ndb_ephemeris_status_t r = EPHE_CAND_MISMATCH;
  ephemeris_t existing;
  ndb_ephemeris_read(new->sid, &existing);

  chMtxLock(&cand_list_access);

  if (!existing.valid) {
    chMtxUnlock(&cand_list_access);
    return EPHE_NEW_TRUSTED;
  }

  s16 cand_idx = ndb_ephe_find_candidate(new);

  /* Ephemeris for this SV was stored to the database already */
  if (memcmp(&existing, new, sizeof(ephemeris_t)) == 0) {
    /* New one is identical to the one in DB, no need to do anything */
    if (cand_idx != -1)
      ndb_ephe_release_candidate(cand_idx);
    chMtxUnlock(&cand_list_access);
    return EPHE_IDENTICAL;
  }

  if (cand_idx != -1) {
    /* Candidate was added already */
    r = memcmp(&ephe_candidates[cand_idx].ephe, new, sizeof(ephemeris_t)) == 0 ?
        EPHE_NEW_TRUSTED : EPHE_CAND_MISMATCH;
    ndb_ephe_release_candidate(cand_idx);
  } else {
    /* New one is not in candidate list yet, try to put it
     * to an empty slot */
    ndb_ephe_try_adding_candidate(new);
    r = EPHE_NEW_CANDIDATE;
  }

  chMtxUnlock(&cand_list_access);
  return r;
}

enum ndb_op_code ndb_ephemeris_store(ephemeris_t *e, enum ndb_data_source src)
{
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_DS_RECEIVER == src) {
    switch (ndb_get_ephemeris_status(e)) {
      case EPHE_IDENTICAL:
        return NDB_ERR_NONE;
      case EPHE_NEW_TRUSTED:
      {
        u16 idx = sid_to_global_index(e->sid);
        return ndb_update(e, src, &ndb_ephemeris_md[idx]);
      }
      case EPHE_NEW_CANDIDATE:
      case EPHE_CAND_MISMATCH:
        return NDB_ERR_UNRELIABLE_DATA;
    }
  } else if (NDB_DS_SBP == src) {
    u8 v, h;
    gps_time_t toe;
    u32 fi;
    ndb_ephemeris_info(e->sid, &v, &h, &toe, &fi);
    if (!v || gpsdifftime(&e->toe, &toe) > 0) {
    /* If local ephemeris is not valid or received one is newer then
     * save the received one. */
      log_debug_sid(e->sid,
                    "Saving ephemeris received over SBP v:%d [%d,%d] vs [%d,%d]",
                    (int)v, toe.wn, toe.tow, e->toe.wn, e->toe.tow);
      u16 idx = sid_to_global_index(e->sid);
      return ndb_update(e, src, &ndb_ephemeris_md[idx]);
    }
    return NDB_ERR_NONE;
  }
  assert(!"ndb_ephemeris_store()");
  return NDB_ERR_ALGORITHM_ERROR;
}

enum ndb_op_code ndb_ephemeris_info(gnss_signal_t sid, u8* v, u8* h,
                                    gps_time_t* toe, u32* fit_interval)
{
  assert(v != NULL);
  assert(h != NULL);
  assert(toe != NULL);
  assert(fit_interval != NULL);
  u16 idx = sid_to_global_index(sid);
  ndb_lock();
  *v = ndb_ephemeris[idx].valid;
  *h = ndb_ephemeris[idx].healthy;
  *toe = ndb_ephemeris[idx].toe;
  *fit_interval = ndb_ephemeris[idx].fit_interval;
  ndb_unlock();
  return NDB_ERR_NONE;
}

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_ephemeris_sbp_update()
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
