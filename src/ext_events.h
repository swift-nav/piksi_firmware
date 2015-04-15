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

#ifndef SWIFTNAV_EXT_EVENTS_H
#define SWIFTNAV_EXT_EVENTS_H

typedef enum {
  TRIG_NONE    = 0x00,
  TRIG_RISING  = 0x01,
  TRIG_FALLING = 0x02,
  TRIG_BOTH    = 0x03
} ext_event_trigger_t;

void ext_event_setup(void);
void ext_event_service(void);

#endif  // SWIFTNAV_EXT_EVENTS_H
