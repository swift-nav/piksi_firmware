/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/logging.h>
#include <libsbp/navigation.h>
#include <libsbp/ext_events.h>

#include "./nap_common.h"
#include "../../settings.h"
#include "../../timing.h"
#include "../../sbp.h"
#include "../../sbp_utils.h"

/** \defgroup ext_events External Events
 * Capture accurate timestamps of external pin events
 * \{ */

typedef enum {
  NONE    = 0x00,
  RISING  = 0x01,
  FALLING = 0x02,
  BOTH    = 0x03
} trigger_type_t;
static trigger_type_t trigger = NONE;

static bool trigger_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val))
  {
    u8 v[5]={0};
    v[4] = trigger;
    nap_xfer_blocking(NAP_REG_EXT_EVENT_TIME, 5, v, v);
    return true;
  }
  return false;
}

/** Set up the external event detection system
 *
 * Informs the NAP of the desired trigger mode, and registers a settings callback
 * to update the NAP if the trigger mode is changed.
 *
 */
void ext_event_setup(void)
{
  static const char const *trigger_enum[] = {"None", "Rising", "Falling", "Both", NULL};
  static struct setting_type trigger_setting;
  int TYPE_TRIGGER = settings_type_register_enum(trigger_enum, &trigger_setting);
  SETTING_NOTIFY("ext_events", "edge_trigger", trigger, TYPE_TRIGGER, trigger_changed);
}

/** Service an external event interrupt
 *
 * Read the trigger edge, pin and time from the NAP.  Simultaneously refresh the NAP
 * trigger mode and clear the IRQ.
 *
 */
void ext_event_service(void)
{
  union {
    struct __attribute__((packed)) {
      u8 edge_pin;
      u32 time;
    } d;
    u8 b[5];
  } v;
  v.b[4] = trigger;  /* We have to reset the trigger for next time */
  nap_xfer_blocking(NAP_REG_EXT_EVENT_TIME, 5, v.b, v.b);

  /* Extract the time of the event */
  u64 event_nap_time = __builtin_bswap32(v.d.time);

  /* We have to infer the most sig word (i.e. # of 262-second rollovers) */
  u64 tc = nap_timing_count();
  if ((tc & 0xFFFFFFFF) < event_nap_time)  /* Rollover occurred since event */
    tc -= UINT64_C(0x100000000);
  event_nap_time |= tc & UINT64_C(0xFFFFFFFF00000000);

  /* Prepare the MSG_EXT_EVENT */
  msg_ext_event_t msg;
  msg.flags = (v.d.edge_pin & 0x80) ? (1<<0) : (0<<0);
  if (time_quality == TIME_FINE)
    msg.flags |= (1 << 1);
  msg.pin = v.d.edge_pin & 0x0F;

  /* Convert to the SBP convention of rounded ms + signed ns residual */
  gps_time_t gpst = rx2gpstime(event_nap_time);
  msg_gps_time_t mgt;
  sbp_make_gps_time(&mgt, &gpst, 0);
  msg.wn = mgt.wn;
  msg.tow = mgt.tow;
  msg.ns = mgt.ns;

  sbp_send_msg(SBP_MSG_EXT_EVENT, sizeof(msg), (u8 *)&msg);
}

/** \} */
