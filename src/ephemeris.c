/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
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

#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_ms   (15 * 1000)
#define EPHEMERIS_MESSAGE_SPACING_ms          (200)

MUTEX_DECL(es_mutex);
static ephemeris_t gps_l1ca_es[NUM_SATS_GPS] _CCM;
static ephemeris_t sbas_l1ca_es[NUM_SATS_SBAS] _CCM;
static ephemeris_t gps_l1ca_es_candidate[NUM_SATS_GPS] _CCM;
static ephemeris_t sbas_l1ca_es_candidate[NUM_SATS_SBAS] _CCM;

static WORKING_AREA_CCM(wa_ephemeris_thread, 1400);

static void transmit_ephes(ephemeris_t *es, u8 len)
{
  for (u32 i=0; i<len; i++) {
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
}

static msg_t ephemeris_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("ephemeris");

  systime_t tx_epoch = chTimeNow();
  while (1) {
    transmit_ephes(gps_l1ca_es, NUM_SATS_GPS);
    transmit_ephes(sbas_l1ca_es, NUM_SATS_SBAS);

    // wait for the next transmit epoch
    tx_epoch += MS2ST(EPHEMERIS_TRANSMIT_EPOCH_SPACING_ms);
    chThdSleepUntil(tx_epoch);
  }

  return 0;
}

static ephemeris_t * select_ephemeris(gnss_signal_t const *sid)
{
  ephemeris_t *ephe = NULL;
  bool valid = sid_valid(*sid);
  assert(valid);
  if (valid) {
    switch (sid_to_constellation(*sid)) {
    case CONSTELLATION_GPS:
      ephe = &gps_l1ca_es[sid_to_index(*sid, INDEX_SAT_IN_CONS)];
      break;
    case CONSTELLATION_SBAS:
      ephe = &sbas_l1ca_es[sid_to_index(*sid, INDEX_SAT_IN_CONS)];
      break;
    default:
      assert("invalid constellation");
      break;
    }
  }
  return ephe;
}

static ephemeris_t * select_ephemeris_candidate(gnss_signal_t const *sid)
{
  ephemeris_t *ephe = NULL;
  bool valid = sid_valid(*sid);
  assert(valid);
  if (valid) {
    switch (sid_to_constellation(*sid)) {
    case CONSTELLATION_GPS:
      ephe = &gps_l1ca_es_candidate[sid_to_index(*sid, INDEX_SAT_IN_CONS)];
      break;
    case CONSTELLATION_SBAS:
      ephe = &sbas_l1ca_es_candidate[sid_to_index(*sid, INDEX_SAT_IN_CONS)];
      break;
    default:
      assert("invalid constellation");
      break;
    }
  }
  return ephe;
}

void ephemeris_new(ephemeris_t *e)
{
  char buf[SID_STR_LEN_MAX];
  ephemeris_lock();
  assert(sid_valid(e->sid));
  sid_to_string(buf, sizeof(buf), e->sid);

  gps_time_t t = get_current_time();
  ephemeris_t *selected_e = select_ephemeris(&e->sid);
  ephemeris_t *selected_e_candidate = select_ephemeris_candidate(&e->sid);
  if (!ephemeris_valid(selected_e, &t)) {
    /* Our currently used ephemeris is bad, so we assume this is better. */
    log_info("New untrusted ephemeris for %s", buf);
    *selected_e = *selected_e_candidate = *e;
  } else if (ephemeris_equal(selected_e_candidate, e)) {
    /* The received ephemeris matches our candidate, so we trust it. */
    log_info("New trusted ephemeris for %s", buf);
    *selected_e = *e;
  } else {
    /* This is our first reception of this new ephemeris, so treat it with
     * suspicion and call it the new candidate. */
    log_info("New ephemeris candidate for %s", buf);
    *selected_e_candidate = *e;
  }
  ephemeris_unlock();
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
  memset(gps_l1ca_es_candidate, 0, sizeof(gps_l1ca_es_candidate));
  memset(gps_l1ca_es, 0, sizeof(gps_l1ca_es));
  memset(sbas_l1ca_es_candidate, 0, sizeof(sbas_l1ca_es_candidate));
  memset(sbas_l1ca_es, 0, sizeof(sbas_l1ca_es));
  for (u32 i=0; i<NUM_SATS_GPS; i++) {
    gps_l1ca_es[i].sid = construct_sid(CODE_GPS_L1CA, i + 1);
  }
  for (u32 i=0; i<NUM_SATS_SBAS; i++) {
    sbas_l1ca_es[i].sid = construct_sid(CODE_SBAS_L1CA, i + 1);
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
  Mutex *m = chMtxUnlock();
  assert(m == &es_mutex);
}

ephemeris_t *ephemeris_get(gnss_signal_t sid)
{
  return select_ephemeris(&sid);
}
