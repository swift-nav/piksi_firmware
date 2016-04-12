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
#include "board/nap/nap_conf.h"
#include "board/frontend.h"
#include "sbp.h"
#include "init.h"
#include "manage.h"
#include "track.h"
#include "track_gps_l1ca.h"
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
#include "decode_gps_l1.h"
#include "signal.h"

extern void ext_setup(void);

/** Compare version strings.
 * Compares a version of the form 'vX.Y-Z-'. If the first character of the
 * version is not 'v' then that string will be considered older than any
 * version string starting with 'v'. Two strings neither starting with 'v' will
 * compare equal.
 *
 * \param a First version string
 * \param b Second version string
 * \return `1` if `a > b`, `-1` if `b > a`, `0` if `a == b`
 */
s8 compare_version(const char *a, const char *b)
{
  if (a[0] != 'v') {
    if (b[0] != 'v') {
      /* Both have old style version strings, no way to compare. */
      return 0;
    } else {
      /* a has an old style version string, so is older. */
      return -1;
    }
  }

  if (b[0] != 'v') {
    /* b has an old style version string, so is older. */
    return 1;
  }

  char buff[5];
  memset(buff, 0, 5);

  /* Skip initial 'v'. */
  a++; b++;

  /* Extract the major version numbers. */
  u8 major_span = strchr(a, '.') - a;
  strncpy(buff, a, major_span);
  u8 major_a = atoi(buff);
  a += major_span + 1;
  memset(buff, 0, 5);

  major_span = strchr(b, '.') - b;
  strncpy(buff, b, major_span);
  u8 major_b = atoi(buff);
  b += major_span + 1;
  memset(buff, 0, 5);

  if (major_a != major_b) {
    return (major_a < major_b) ? -1 : 1;
  }

  u8 commit_a = 0;
  u8 commit_b = 0;
  u8 minor_a, minor_b;

  /* Check if we have a commit number. */
  if (strchr(a, '-')) {
    /* Extract the minor version numbers. */
    u8 minor_span = strchr(a, '-') - a;
    strncpy(buff, a, minor_span);
    minor_a = atoi(buff);
    a += minor_span + 1;
    memset(buff, 0, 5);

    /* Extract the commit numbers. */
    commit_a = atoi(a);
  } else {
    minor_a = atoi(a);
  }

  /* Check if we have a commit number. */
  if (strchr(b, '-')) {
    /* Extract the minor version numbers. */
    u8 minor_span = strchr(b, '-') - b;
    strncpy(buff, b, minor_span);
    minor_b = atoi(buff);
    b += minor_span + 1;

    /* Extract the commit numbers. */
    commit_b = atoi(b);
  } else {
    minor_b = atoi(b);
  }

  if (minor_a != minor_b) {
    return (minor_a < minor_b) ? -1 : 1;
  }

  return (commit_a < commit_b) ? -1 : (commit_a > commit_b);
}


int main(void)
{
  halInit();

  /* Kernel initialization, the main() function becomes a thread with
   * priority NORMALPRIO and the RTOS is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  init();
  settings_setup();
  signal_init();

  check_nap_auth();

  static char nap_version_string[64] = {0};
  nap_conf_rd_version_string(nap_version_string);
  log_info("NAP firmware version: %s", nap_version_string);

  /* Check we are running a compatible version of the NAP firmware. */
  const char *required_nap_version = "v0.16";
  if (compare_version(nap_version_string, required_nap_version) < 0) {
    while (1) {
      log_error("NAP firmware version >= %s required, please update!"
                "(instructions can be found at http://docs.swift-nav.com/)",
                required_nap_version);
      chThdSleepSeconds(2);
    }
  }

  static s32 serial_number;
  serial_number = nap_conf_rd_serial_number();

  frontend_setup();
  timing_setup();
  ext_event_setup();
  position_setup();
  track_setup();
  track_gps_l1ca_register();
  decode_setup();
  decode_gps_l1_register();

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

  static struct setting hw_rev = {
    "system_info", "hw_revision", NULL, 0,
    settings_read_only_notify, NULL,
    NULL, false
  };
  hw_rev.addr = (char *)nap_conf_rd_hw_rev_string();
  hw_rev.len = strlen(hw_rev.addr);
  settings_register(&hw_rev, TYPE_STRING);

  READ_ONLY_PARAMETER("system_info", "nap_version", nap_version_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_channels", nap_track_n_channels,
                      TYPE_INT);
  READ_ONLY_PARAMETER("system_info", "nap_fft_index_bits", nap_acq_fft_index_bits, TYPE_INT);

  ephemeris_setup();

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  sbp_send_msg(SBP_MSG_STARTUP, sizeof(startup_flags), (u8 *)&startup_flags);

  while (1) {
    chThdSleepSeconds(60);
  }
}
