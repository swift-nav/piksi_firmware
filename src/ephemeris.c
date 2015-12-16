/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "timing.h"
#include "ephemeris.h"

MUTEX_DECL(es_mutex);
static ephemeris_t es[NUM_SATS] _CCM;
static ephemeris_t es_candidate[NUM_SATS] _CCM;

void ephemeris_new(ephemeris_t *e)
{
  assert(sid_valid(e->sid));

  gps_time_t t = get_current_time();
  u32 index = sid_to_index(e->sid);
  if (!ephemeris_good(&es[index], t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info("New untrusted ephemeris for PRN %02d", e->sid.sat+1);
    ephemeris_lock();
    es[index] = es_candidate[index] = *e;
    ephemeris_unlock();

  } else if (ephemeris_equal(&es_candidate[index], e)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for PRN %02d", e->sid.sat+1);
    ephemeris_lock();
    es[index] = *e;
    ephemeris_unlock();
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for PRN %02d", e->sid.sat+1);
    ephemeris_lock();
    es_candidate[index] = *e;
    ephemeris_unlock();
  }
}

static void ephemeris_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_ephemeris_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t e;
  unpack_ephemeris((msg_ephemeris_t *)msg, &e);
  if (!sid_valid(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  ephemeris_new(&e);
}

void ephemeris_setup(void)
{
  memset(es_candidate, 0, sizeof(es_candidate));
  memset(es, 0, sizeof(es));
  for (u32 i=0; i<NUM_SATS; i++) {
    es[i].sid = sid_from_index(i);
  }

  static sbp_msg_callbacks_node_t ephemeris_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS,
    &ephemeris_msg_callback,
    &ephemeris_msg_node
  );
}

void ephemeris_lock(void)
{
  chMtxLock(&es_mutex);
}

void ephemeris_unlock(void)
{
  Mutex *m = chMtxUnlock();
  assert(m == &es_mutex);
}

ephemeris_t *ephemeris_get(gnss_signal_t sid)
{
  assert(sid_valid(sid));
  return &es[sid_to_index(sid)];
}
