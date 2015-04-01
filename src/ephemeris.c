/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
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

#include "sbp.h"
#include "track.h"
#include "ephemeris.h"

/* TODO: Think about thread safety when updating ephemerides. */
ephemeris_t es[MAX_SATS] _CCM;
ephemeris_t es_old[MAX_SATS] _CCM;

static WORKING_AREA_CCM(wa_nav_msg_thread, 3000);
static msg_t nav_msg_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("nav msg");

  memset(es, 0, sizeof(es));
  for (u8 i=0; i<32; i++) {
    es[i].prn = i;
  }

  while (TRUE) {

    /* TODO: This should be trigged by a semaphore from the tracking loop, not
     * just ran periodically. */


    for (u8 i=0; i<nap_track_n_channels; i++) {
      chThdSleepMilliseconds(100);
      tracking_channel_t *ch = &tracking_channel[i];

      /* Check if there is a new nav msg subframe to process.
       * TODO: move this into a function */
      if ((ch->state != TRACKING_RUNNING) ||
          (ch->nav_msg.subframe_start_index == 0))
        continue;

      /* Save old ephemeris before potentially updating. */
      memcpy(&es_old[ch->prn], &es[ch->prn], sizeof(ephemeris_t));

      __asm__("CPSID i;");
      s8 ret = process_subframe(&ch->nav_msg, &es[ch->prn]);
      __asm__("CPSIE i;");

      if (ret < 0) {
        log_info("PRN %02d ret %d\n", ch->prn+1, ret);
      } else if (ret == 1) {
        /* Decoded a new ephemeris. */

        if (memcmp(&es[ch->prn], &es_old[ch->prn], sizeof(ephemeris_t))) {
          log_info("New ephemeris for PRN %02d\n", ch->prn+1);
        }

        if (!es[ch->prn].healthy) {
          log_info("PRN %02d unhealthy\n", ch->prn+1);
        } else {
          sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(ephemeris_t),
                       (u8 *)&es[ch->prn]);
        }
      }
    }
  }

  return 0;
}

void ephemeris_setup(void)
{
  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread),
                    NORMALPRIO-1, nav_msg_thread, NULL);
}

