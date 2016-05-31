/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdlib.h>
#include <string.h>

#include <libsbp/system.h>
#include <libswiftnav/logging.h>

#include <hal.h>
#include <ch.h>

#include "peripherals/leds.h"
#include "board/frontend.h"
#include "sbp.h"
#include "init.h"
#include "manage.h"
#include "track.h"
#include "timing.h"
#include "ext_events.h"
#include "solution.h"
#include "base_obs.h"
#include "position.h"
#include "system_monitor.h"
#include "simulator.h"
#include "settings.h"
#include "sbp_fileio.h"
#include "ephemeris.h"
#include "pps.h"
#include "decode.h"
#include "signal.h"
#include "version.h"

extern void ext_setup(void);

int main(void)
{
  halInit();

  /* Kernel initialization, the main() function becomes a thread with
   * priority NORMALPRIO and the RTOS is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  pre_init();

  usarts_setup();
  static s32 serial_number;
  serial_number = serial_number_get();
  if (serial_number < 0) {
    /* TODO: Handle this properly! */
    serial_number = 0x2222;
  }
  sbp_setup(serial_number);

  init();
  settings_setup();
  signal_init();
  ephemeris_setup();

  static char hw_revision_string[64] = {0};
  hw_revision_string_get(hw_revision_string);
  log_info("HW revision: %s", hw_revision_string);

  static char nap_version_string[64] = {0};
  nap_version_string_get(nap_version_string);
  log_info("NAP firmware version: %s", nap_version_string);

  frontend_setup();
  timing_setup();
  ext_event_setup();
  position_setup();
  track_setup();
  decode_setup();

  manage_acq_setup();
  manage_track_setup();
  system_monitor_setup();
  base_obs_setup();
  solution_setup();

  simulator_setup();

  sbp_fileio_setup();
  ext_setup();
  pps_setup();

  READ_ONLY_PARAMETER("system_info", "serial_number", serial_number, TYPE_INT);
  READ_ONLY_PARAMETER("system_info", "firmware_version", GIT_VERSION,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "firmware_built", __DATE__ " " __TIME__,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "hw_revision", hw_revision_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_version", nap_version_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_channels", nap_track_n_channels,
                      TYPE_INT);

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  sbp_send_msg(SBP_MSG_STARTUP, sizeof(startup_flags), (u8 *)&startup_flags);

  while (1) {
    chThdSleepSeconds(60);
  }
}
