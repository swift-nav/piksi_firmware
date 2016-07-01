/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <libswiftnav/signal.h>
#include <ch.h>

#include "cnav_msg_storage.h"

static MUTEX_DECL(type30_mutex);
static cnav_msg_type_30_t type_30_store[NUM_SATS_GPS];

void cnav_msg_type30_put(const cnav_msg_t *msg)
{
  if (CNAV_MSG_TYPE_30 != msg->msg_id)
    return;

  if (msg->prn > NUM_SATS_GPS)
    return;

  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, msg->prn);

  u16 idx = sid_to_code_index(sid);
  chMtxLock(&type30_mutex);
  type_30_store[idx] = msg->data.type_30;
  chMtxUnlock(&type30_mutex);
}

bool cnav_msg_type30_get(gnss_signal_t sid, cnav_msg_type_30_t *msg)
{
  bool res = false;

  if (sid_valid(sid) && sid_to_constellation(sid) == CONSTELLATION_GPS) {
    u16 idx = sid_to_code_index(sid);
    chMtxLock(&type30_mutex);
    *msg = type_30_store[idx];
    chMtxUnlock(&type30_mutex);
    res = true;
  }

  return res;
}
