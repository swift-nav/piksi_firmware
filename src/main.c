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

#include <stdio.h>
#include <string.h>

#include <libswiftnav/sbp_messages.h>

#include <ch.h>

#include "board/leds.h"
#include "board/nap/nap_conf.h"
#include "board/max2769.h"
#include "sbp.h"
#include "init.h"
#include "manage.h"
#include "track.h"
#include "timing.h"
#include "solution.h"
#include "position.h"
#include "system_monitor.h"
#include "simulator.h"
#include "settings.h"

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 130944000
#endif

/* TODO: Think about thread safety when updating ephemerides. */
ephemeris_t es[32];
ephemeris_t es_old[32];

/* Required by exit() which is called from BLAS/LAPACK. */
void _fini(void)
{
  return;
}

static WORKING_AREA_CCM(wa_nav_msg_thread, 4096);
static msg_t nav_msg_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("nav msg");
  while (TRUE) {
    chThdSleepMilliseconds(1000);

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    /* TODO: This should be trigged by a semaphore from the tracking loop, not
     * just ran periodically. */

    memcpy(es_old, es, sizeof(es));

    for (u8 i=0; i<nap_track_n_channels; i++) {
      if (tracking_channel[i].state == TRACKING_RUNNING &&
          tracking_channel[i].nav_msg.subframe_start_index) {

        __asm__("CPSID i;");
        s8 ret = process_subframe(&tracking_channel[i].nav_msg,
                                  &es[tracking_channel[i].prn]);
        __asm__("CPSIE i;");

        if (ret < 0)
          printf("PRN %02d ret %d\n", tracking_channel[i].prn+1, ret);

        if (ret == 1 && !es[tracking_channel[i].prn].healthy)
          printf("PRN %02d unhealthy\n", tracking_channel[i].prn+1);

        if (memcmp(&es[tracking_channel[i].prn],
                   &es_old[tracking_channel[i].prn], sizeof(ephemeris_t))) {

          printf("New ephemeris for PRN %02d\n", tracking_channel[i].prn+1);

          /* TODO: This is a janky way to set the time... */
          gps_time_t t;
          t.wn = es[tracking_channel[i].prn].toe.wn;
          t.tow = tracking_channel[i].TOW_ms / 1000.0;
          if (gpsdifftime(t, es[tracking_channel[i].prn].toe) > 2*24*3600)
            t.wn--;
          else if (gpsdifftime(t, es[tracking_channel[i].prn].toe) < 2*24*3600)
            t.wn++;
          /*set_time(TIME_COARSE, t);*/

        }
      }
    }
  }

  return 0;
}

int main(void)
{
  /* Initialise SysTick timer that will be used as the ChibiOS kernel tick
   * timer. */
  STBase->RVR = SYSTEM_CLOCK / CH_FREQUENCY - 1;
  STBase->CVR = 0;
  STBase->CSR = CLKSOURCE_CORE_BITS | ENABLE_ON_BITS | TICKINT_ENABLED_BITS;

  /* Kernel initialization, the main() function becomes a thread and the RTOS
   * is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  init(1);
  settings_setup();
  usarts_setup();


  u8 nap_git_hash[20];
  static char nap_version_string[64] = {0};

  /* Read out NAP git hash and construct version string. */
  /* TODO: Change NAP version in the M25 to just be a string. */
  nap_conf_rd_git_hash(nap_git_hash);
  for (u8 i=0; i<20; i++)
    snprintf(&nap_version_string[2*i], 3, "%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    strcpy(&nap_version_string[40], " (unclean)");

  max2769_setup();
  timing_setup();
  position_setup();

  manage_acq_setup();
  manage_track_setup();
  system_monitor_setup();
  solution_setup();

  simulator_setup();

  READ_ONLY_PARAMETER("system_info", "firmware_version", GIT_VERSION, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "firmware_built", __DATE__ " " __TIME__, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_version", nap_version_string, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_channels", nap_track_n_channels, TYPE_INT);

  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread),
                    NORMALPRIO-1, nav_msg_thread, NULL);

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  sbp_send_msg(SBP_STARTUP, sizeof(startup_flags), (u8 *)&startup_flags);

  while (1) {
    chThdSleepSeconds(60);
  }
}

