/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Vlad Ungureanu <vvu@vdev.ro>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <ch.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/bits.h>
#include <libswiftnav/gpstime.h>
#include <fec.h>

#include "decoder.h"
#include "system_monitor.h"
#include "timing.h"
#include "error.h"
#include "main.h"
#include "track.h"
#include "position.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"
#include "sbp_utils.h"
#include "sbp.h"

struct v27 decoder;
decision_t decisions[DECISION_SIZE] _CCM;

extern l1_sbas_nav_msg_t sbas_nav_msgs[WAAS_SATS];

extern Mailbox sbas_mb;

static WORKING_AREA_CCM(wa_decoder_thread, 2000);
static msg_t decoder_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decoder");
  while (TRUE) {
    watchdog_notify(WD_NOTIFY_DECODER);

    for (u8 i = 0; i < WAAS_SATS; i++) {
      __attribute__ ((unused)) l1_sbas_nav_msg_t *nav_msg = &sbas_nav_msgs[i];

      /*
       *Skip if it is not our prn!
       */
      if (nav_msg->sid.prn != 137)
        continue;

      u8 bit;
      if (chMBFetch(&sbas_mb, (msg_t *)&bit, TIME_IMMEDIATE) == RDY_OK) {
        nav_msg->symbols[nav_msg->symbol_count++] = bit;
        if (nav_msg->symbol_count == SBAS_NAVFLEN) {
          log_info("Decoder started!");

          /*
           *Do actual decoding normally.
           */
          init_viterbi27(&decoder, 0);
          update_viterbi27_blk(&decoder, nav_msg->symbols, SBAS_BITS_UPDATE);
          chainback_viterbi27(&decoder, nav_msg->decoded, SBAS_BITS_CHAINBACK, 0);

          /*
           *Try to see if we have valid msgs.
           */
          __attribute__ ((unused)) ephemeris_xyz_t e;
          __attribute__ ((unused)) sbas_almanac_t alm;
          l1_sbas_process_subframe(nav_msg, &e, &alm);

          /*
           *Save last 12 bits for next decoding round.
           */
          for(u8 j = 0; j < BITS_TO_KEEP; j++)
            nav_msg->symbols[j] = nav_msg->symbols[SBAS_NAVFLEN + j - BITS_TO_KEEP];


          nav_msg->symbol_count = BITS_TO_KEEP;
          log_info("Decoder finished!\n");
        }
      }

    }
  }

  return 0;
}

void decoder_setup(void)
{
  set_decisions_viterbi27(&decoder, decisions);
  chThdCreateStatic(wa_decoder_thread, sizeof(wa_decoder_thread),
                    DECODER_THREAD_PRIORITY, decoder_thread, NULL);
}

