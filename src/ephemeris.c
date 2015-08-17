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

#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "timing.h"
#include "ephemeris.h"

MUTEX_DECL(es_mutex);
ephemeris_t es[MAX_SATS] _CCM;
static ephemeris_t es_candidate[MAX_SATS] _CCM;

static void ephemeris_new(ephemeris_t *e)
{
  gps_time_t t = get_current_time();
  if (!ephemeris_good(&es[e->sid.prn], t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info("New untrusted ephemeris for PRN %02d", e->sid.prn+1);
    chMtxLock(&es_mutex);
    es[e->sid.prn] = es_candidate[e->sid.prn] = *e;
    chMtxUnlock();

  } else if (ephemeris_equal(&es_candidate[e->sid.prn], e)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for PRN %02d", e->sid.prn+1);
    chMtxLock(&es_mutex);
    es[e->sid.prn] = *e;
    chMtxUnlock();
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for PRN %02d", e->sid.prn+1);
    chMtxLock(&es_mutex);
    es_candidate[e->sid.prn] = *e;
    chMtxUnlock();
  }
}

static WORKING_AREA_CCM(wa_nav_msg_thread, 3000);
static msg_t nav_msg_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("nav msg");

  while (TRUE) {

    /* TODO: This should be trigged by a semaphore from the tracking loop, not
     * just ran periodically. */


    for (u8 i=0; i<nap_track_n_channels; i++) {
      chThdSleepMilliseconds(100);
      tracking_channel_t *ch = &tracking_channel[i];
      ephemeris_t e = {.sid.prn = ch->sid.prn};

      /* Check if there is a new nav msg subframe to process.
       * TODO: move this into a function */
      if ((ch->state != TRACKING_RUNNING) ||
          (ch->nav_msg.subframe_start_index == 0))
        continue;

      /* Decode ephemeris to temporary struct */
      __asm__("CPSID i;");
      s8 ret = process_subframe(&ch->nav_msg, &e);
      __asm__("CPSIE i;");

      if (ret <= 0)
        continue;

      /* Decoded a new ephemeris. */
      ephemeris_new(&e);

      if (!es[ch->sid.prn].healthy) {
        log_info("PRN %02d unhealthy", ch->sid.prn+1);
      } else {
        msg_ephemeris_t msg;
        pack_ephemeris(&es[ch->sid.prn], &msg);
        sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *)&msg);
      }
    }
  }

  return 0;
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
  if (e.sid.prn >= MAX_SATS) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  ephemeris_new(&e);
}

void ephemeris_setup(void)
{
  memset(es_candidate, 0, sizeof(es_candidate));
  memset(es, 0, sizeof(es));
  for (u8 i=0; i<32; i++) {
    es[i].sid.prn = i;
  }

  static sbp_msg_callbacks_node_t ephemeris_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS,
    &ephemeris_msg_callback,
    &ephemeris_msg_node
  );

  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread),
                    NORMALPRIO-1, nav_msg_thread, NULL);
}

