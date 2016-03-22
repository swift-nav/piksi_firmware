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
#include "signal.h"
#include "ndb.h"

static ephemeris_t es_candidate[PLATFORM_SIGNAL_COUNT];

void ephemeris_new(ephemeris_t *e)
{
  assert(sid_supported(e->sid));

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), e->sid);

  if (!e->valid) {
    log_error("Invalid ephemeris for %s", buf);
    return;
  }

  u32 index = sid_to_global_index(e->sid);

  if(ephemeris_equal(&es_candidate[index], e)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for %s", buf);
    if (ndb_ephemeris_store(e, NDB_DS_RECEIVER) != NDB_ERR_NONE)
      log_error("Error storing ephemeris for %s", buf);
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for %s", buf);
    es_candidate[index] = *e;
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
  if (!sid_supported(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }
  /* We trust epehemeris that we received over SBP, so save it to NDB right
   * away. If we receive new one from the sky twice it will replace it. */
  ndb_ephemeris_store(&e, NDB_DS_SBP);
}

void ephemeris_setup(void)
{
  memset(es_candidate, 0, sizeof(es_candidate));
  static sbp_msg_callbacks_node_t ephemeris_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS,
    &ephemeris_msg_callback,
    &ephemeris_msg_node
  );
}
