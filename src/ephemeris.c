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

#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_ms   (15 * 1000)
#define EPHEMERIS_MESSAGE_SPACING_ms          (200)

MUTEX_DECL(es_mutex);
static ephemeris_t es[PLATFORM_SIGNAL_COUNT] _CCM;
static ephemeris_t es_candidate[PLATFORM_SIGNAL_COUNT] _CCM;

static WORKING_AREA_CCM(wa_ephemeris_thread, 1400);

static void ephemeris_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("ephemeris");

  systime_t tx_epoch = chVTGetSystemTime();
  while (1) {

    for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
      bool success = false;
      const ephemeris_t *e = &es[i];
      gps_time_t t = get_current_time();

      /* Quickly check validity before locking */
      if (ephemeris_valid(e, &t)) {
        ephemeris_lock();
        /* Now that we are locked, reverify validity and transmit */
        if (ephemeris_valid(e, &t)) {
          msg_ephemeris_t msg;
          pack_ephemeris(e, &msg);
          sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *)&msg);
          success = true;
        }
        ephemeris_unlock();
      }

      if (success) {
        chThdSleep(MS2ST(EPHEMERIS_MESSAGE_SPACING_ms));
      }
    }

    // wait for the next transmit epoch
    tx_epoch += MS2ST(EPHEMERIS_TRANSMIT_EPOCH_SPACING_ms);
    chThdSleepUntil(tx_epoch);
  }
}

void ephemeris_new(ephemeris_t *e)
{
  assert(sid_supported(e->sid));

  gps_time_t t = get_current_time();
  u32 index = sid_to_global_index(e->sid);
  if (!ephemeris_valid(&es[index], &t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info_sid(e->sid, "New untrusted ephemeris");
    ephemeris_lock();
    es[index] = es_candidate[index] = *e;
    ephemeris_unlock();

  } else if (ephemeris_equal(&es_candidate[index], e)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info_sid(e->sid, "New trusted ephemeris");
    ephemeris_lock();
    es[index] = *e;
    ephemeris_unlock();
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info_sid(e->sid, "New ephemeris candidate");
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
  if (!sid_supported(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  ephemeris_new(&e);
}

void ephemeris_setup(void)
{
  memset(es_candidate, 0, sizeof(es_candidate));
  memset(es, 0, sizeof(es));
  for (u32 i=0; i<PLATFORM_SIGNAL_COUNT; i++) {
    es[i].sid = sid_from_global_index(i);
  }

  static sbp_msg_callbacks_node_t ephemeris_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS,
    &ephemeris_msg_callback,
    &ephemeris_msg_node
  );

  chThdCreateStatic(wa_ephemeris_thread, sizeof(wa_ephemeris_thread),
                    NORMALPRIO-10, ephemeris_thread, NULL);
}

void ephemeris_lock(void)
{
  chMtxLock(&es_mutex);
}

void ephemeris_unlock(void)
{
  chMtxUnlock(&es_mutex);
}

ephemeris_t *ephemeris_get(gnss_signal_t sid)
{
  assert(sid_supported(sid));

  /* L2CM uses L1CA ephes */
  if (CODE_GPS_L2CM == sid.code)
    sid.code = CODE_GPS_L1CA;

  return &es[sid_to_global_index(sid)];
}
