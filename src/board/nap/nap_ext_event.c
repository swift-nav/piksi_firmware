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
#include <libsbp/ext_events.h>

#include "./nap_common.h"
#include "../../settings.h"

/** \defgroup ext_events External Events
 * Capture accurate timestamps of external pin events
 * \{ */

typedef enum {
  NONE,
  RISING,
  FALLING,
  BOTH
} trigger_type_t;
trigger_type_t trigger = NONE;

static bool trigger_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val))
  {
    u8 v[5]={0};
    v[4] = trigger;
    nap_xfer_blocking(NAP_REG_EXT_EVENT_TIME, 5, v, v);
    log_info("External event trigger setting: %s\n", val);
    return true;
  }
  return false;
}

/** Setup the external event detection system
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
  v.b[4] = trigger;  // Set for next time
  nap_xfer_blocking(NAP_REG_EXT_EVENT_TIME, 5, v.b, v.b);
  u32 event_nap_time = __builtin_bswap32(v.d.time);
  bool edge = v.d.edge_pin & 0x80;
  u8 pin = v.d.edge_pin & 0x0F;
  log_info("%s edge on DEBUG%d @ %u\n", edge ? "Rising" : "Falling",
           pin, (unsigned int)event_nap_time);
}

/** \} */
