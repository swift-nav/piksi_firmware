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

#include "board/nap/nap_common.h"
#include "./settings.h"
#include "./timing.h"
#include "./sbp.h"
#include "./sbp_utils.h"
#include "./ext_events.h"

/** \defgroup ext_events External Events
 * Capture accurate timestamps of external pin events
 * \{ */

static ext_event_trigger_t trigger = TRIG_NONE;

/** Settings callback to inform NAP which trigger mode is desired */
static bool trigger_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val))
  {
    nap_rw_ext_event(NULL, NULL, trigger);
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
  static const char const *trigger_enum[] =
    {"None", "Rising", "Falling", "Both", NULL};
  static struct setting_type trigger_setting;
  int TYPE_TRIGGER = settings_type_register_enum(trigger_enum,
						 &trigger_setting);
  SETTING_NOTIFY("ext_events", "edge_trigger", trigger, TYPE_TRIGGER,
		 trigger_changed);
  /* trigger_changed() will be called at setup time (i.e. immediately) as well
     as if user changes the setting later. */
}

/** Service an external event interrupt
 *
 * When an event occurs (i.e. pin edge) that matches the NAP's trigger
 * condition, the NAP will latch the time, pin number and trigger direction.
 * It will also set an IRQ bit which will lead to an EXTI.  The firmware
 * EXTI handling routine handle_nap_exti() will call this function, which
 * reads out the details and spits them out as an SBP message to our host.
 *
 */
void ext_event_service(void)
{
  u8 event_pin;
  ext_event_trigger_t event_trig;

  /* Read the details, and also clear IRQ + set up for next time */
  u32 event_nap_time = nap_rw_ext_event(&event_pin, &event_trig, trigger);
  
  /* We have to infer the most sig word (i.e. # of 262-second rollovers) */
  union {
    u32 half[2];
    u64 full;
  } tc;
  tc.full = nap_timing_count();
  if (tc.half[0] < event_nap_time)  /* Rollover occurred since event */
    tc.half[1]--;
  tc.half[0] = event_nap_time;

  /* Prepare the MSG_EXT_EVENT */
  msg_ext_event_t msg;
  msg.flags = (event_trig == TRIG_RISING) ? (1<<0) : (0<<0);
  if (time_quality == TIME_FINE)
    msg.flags |= (1 << 1);
  msg.pin = event_pin;

  /* Convert to the SBP convention of rounded ms + signed ns residual */
  gps_time_t gpst = rx2gpstime(tc.full);
  msg_gps_time_t mgt;
  sbp_make_gps_time(&mgt, &gpst, 0);
  msg.wn = mgt.wn;
  msg.tow = mgt.tow;
  msg.ns = mgt.ns;

  sbp_send_msg(SBP_MSG_EXT_EVENT, sizeof(msg), (u8 *)&msg);
}

/** \} */
