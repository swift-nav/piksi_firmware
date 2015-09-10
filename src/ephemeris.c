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

ephemeris_kepler_t l1_es[GPS_L1_SATS] _CCM;
ephemeris_xyz_t sbas_es[WAAS_SATS] _CCM;

static ephemeris_kepler_t l1_es_candidate[GPS_L1_SATS] _CCM;
static ephemeris_xyz_t sbas_es_candidate[WAAS_SATS] _CCM;

static void ephemeris_xyz_new(ephemeris_t *e, signal_t sid)
{
  gps_time_t t = get_current_time();
  ephemeris_xyz_t *e_xyz = &e->ephemeris_xyz[0];
  ephemeris_xyz_t *es = &sbas_es[sbas_sid_to_index(sid)];
  ephemeris_xyz_t *es_candidate = &sbas_es_candidate[sbas_sid_to_index(sid)];

  if (!ephemeris_good(e, sid, t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info("New untrusted ephemeris for PRN %02d", e_xyz->sid.prn+1);
    chMtxLock(&es_mutex);
    *es = *es_candidate = *e_xyz;
    chMtxUnlock();
  } else if (ephemeris_xyz_equal(es, e_xyz)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for PRN %02d", e_xyz->sid.prn+1);
    chMtxLock(&es_mutex);
    *es = *e_xyz;
    chMtxUnlock();
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for PRN %02d", e_xyz->sid.prn+1);
    chMtxLock(&es_mutex);
    *es_candidate = *e_xyz;
    chMtxUnlock();
  }
}

static void ephemeris_kepler_new(ephemeris_t *e, signal_t sid)
{
  gps_time_t t = get_current_time();
  ephemeris_kepler_t *e_kep = &e->ephemeris_kep[0];
  ephemeris_kepler_t *es = &l1_es[sid.prn];
  ephemeris_kepler_t *es_candidate = &l1_es_candidate[sid.prn];

  if (!ephemeris_good(e, sid, t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info("New untrusted ephemeris for PRN %02d", e_kep->sid.prn+1);
    chMtxLock(&es_mutex);
    *es = *es_candidate = *e_kep;
    chMtxUnlock();
  } else if (ephemeris_kepler_equal(es, e_kep)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for PRN %02d", e_kep->sid.prn+1);
    chMtxLock(&es_mutex);
    *es = *e_kep;
    chMtxUnlock();
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for PRN %02d", e_kep->sid.prn+1);
    chMtxLock(&es_mutex);
    *es = *e_kep;
    chMtxUnlock();
  }
}

static void ephemeris_new(ephemeris_t *e, signal_t sid)
{
  if (sid.constellation == GPS_CONSTELLATION) {
    ephemeris_kepler_new(e, sid);
  } else {
    ephemeris_xyz_new(e, sid);
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
      signal_t sid = ch->sid;

      ephemeris_kepler_t e_kep = {
        .sid = sid
      };

      ephemeris_xyz_t e_xyz = {
        .sid = sid
      };

      ephemeris_t e;
      if (sid.constellation == GPS_CONSTELLATION) {
        e.ephemeris_kep = &e_kep;
        e.ephemeris_xyz = NULL;
      } else {
        e.ephemeris_xyz = &e_xyz;
        e.ephemeris_kep = NULL;
      }

      /* Check if there is a new nav msg subframe to process.
       * TODO: move this into a function */
      if (ch->nav_msg.type == L1_LEGACY_NAV) {
        if ((ch->state != TRACKING_RUNNING) ||
            (ch->nav_msg.l1_nav_msg->subframe_start_index == 0))
          continue;
      }

      /* Decode ephemeris to temporary struct */
      __asm__("CPSID i;");
        s8 ret = 0;
        if (ch->nav_msg.type == L1_LEGACY_NAV)
          ret = l1_legacy_process_subframe(ch->nav_msg.l1_nav_msg, &e.ephemeris_kep[0]);
      __asm__("CPSIE i;");

      if (ret <= 0)
        continue;

      /* Decoded a new ephemeris. */
      ephemeris_new(&e, sid);

      u8 health;
      if (sid.constellation == GPS_CONSTELLATION) {
        health = e_kep.healthy;
      } else {
        health = e_xyz.healthy;
      }

      if (!health) {
        log_info("PRN %02d unhealthy", ch->sid.prn+1);
      } else {
        if (sid.constellation == GPS_CONSTELLATION) {
          msg_ephemeris_kepler_t msg;
          pack_ephemeris_kepler(&e_kep, &msg);
          sbp_send_msg(SBP_MSG_EPHEMERIS_KEPLER,
                       sizeof(msg_ephemeris_kepler_t), (u8 *)&msg);
        } else {
          msg_ephemeris_xyz_t msg;
          pack_ephemeris_xyz(&e_xyz, &msg);
          sbp_send_msg(SBP_MSG_EPHEMERIS_XYZ,
                       sizeof(msg_ephemeris_xyz_t), (u8 *)&msg);
        }
      }
    }
  }

  return 0;
}

static void ephemeris_kepler_msg_callback(u16 sender_id, u8 len,
                                          u8 msg[], void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_ephemeris_kepler_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t es;
  ephemeris_kepler_t e_kep;
  unpack_ephemeris_kepler((msg_ephemeris_kepler_t *)msg, &e_kep);
  if (e_kep.sid.prn >= GPS_L1_SATS) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  es.ephemeris_kep = &e_kep;
  es.ephemeris_xyz = NULL;

  ephemeris_new(&es, e_kep.sid);
}


static void ephemeris_xyz_msg_callback(u16 sender_id, u8 len,
                                       u8 msg[], void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_ephemeris_xyz_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t e;
  ephemeris_xyz_t e_xyz;
  unpack_ephemeris_xyz((msg_ephemeris_xyz_t *)msg, &e_xyz);
  if (e_xyz.sid.prn > 138 || e_xyz.sid.prn < 120) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  e.ephemeris_xyz = &e_xyz;
  e.ephemeris_kep = NULL;

  ephemeris_new(&e, e_xyz.sid);
}

void l1_ephemeris_setup(void)
{
  memset(l1_es_candidate, 0, sizeof(l1_es_candidate));
  memset(l1_es, 0, sizeof(l1_es));
  for (u8 i=0; i<GPS_L1_SATS; i++) {
    l1_es[i].sid.prn = i;
  }
}

void sbas_ephemeris_setup(void)
{
  memset(sbas_es_candidate, 0, sizeof(sbas_es_candidate));
  memset(sbas_es, 0, sizeof(sbas_es));

  for (u8 i=0; i<WAAS_SATS; i++) {
    sbas_es[i].sid = sbas_index_to_sid(i);
  }
}

void ephemeris_setup(void)
{
  l1_ephemeris_setup();
  sbas_ephemeris_setup();

  static sbp_msg_callbacks_node_t ephemeris_kepler_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS_KEPLER,
    &ephemeris_kepler_msg_callback,
    &ephemeris_kepler_msg_node
  );

  static sbp_msg_callbacks_node_t ephemeris_xyz_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS_XYZ,
    &ephemeris_xyz_msg_callback,
    &ephemeris_xyz_msg_node
  );

  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread),
                    NORMALPRIO-1, nav_msg_thread, NULL);
}

