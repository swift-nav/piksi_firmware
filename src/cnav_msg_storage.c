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

#include <libswiftnav/signal.h>
#include <ch.h>

#include "cnav_msg_storage.h"

MUTEX_DECL(type30_mutex);
static cnav_msg_type_30_t type_30_store[NUM_SATS_GPS];

void cnav_msg_type30_put(cnav_msg_t *msg)
{
  if (CNAV_MSG_TYPE_30 != msg->msg_id)
    return;
  if (msg->prn >= NUM_SATS_GPS)
    return;

  chMtxLock(&type30_mutex);
  type_30_store[msg->prn] = msg->data.type_30;
  chMtxUnlock(&type30_mutex);
}

void cnav_msg_type30_get(u8 prn, cnav_msg_type_30_t *msg)
{
  if (prn >= NUM_SATS_GPS)
    return;

  chMtxLock(&type30_mutex);
  *msg = type_30_store[prn];
  chMtxUnlock(&type30_mutex);
}
