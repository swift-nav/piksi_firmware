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
#include "ephemeris.h"
#include "system_monitor.h"
#include "timing.h"
#include "main.h"
#include "track.h"
#include "position.h"
#include "sbp_utils.h"
#include "sbp.h"

struct v27 decoder _CCM;
decision_t decisions[DECISION_SIZE] _CCM;

extern time_quality_t time_quality;
extern sbas_almanac_t sbas_almanac[WAAS_SATS];
extern gnss_solution position_solution;
extern l1_sbas_nav_msg_t sbas_nav_msgs[WAAS_SATS];
extern Mailbox sbas_mb[3];

static WORKING_AREA_CCM(wa_decoder_thread, 1550);
static msg_t decoder_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decoder");
  while (TRUE) {
    watchdog_notify(WD_NOTIFY_DECODER);

    for (u8 i = 0; i < WAAS_SATS; i++) {
      l1_sbas_nav_msg_t *nav_msg = &sbas_nav_msgs[i];

      /*
       *Skip nav_msg if not initialised (mailbox may have a problem)
       */
      if (nav_msg->init == 0)
        continue;

      u8 bit;
      if (chMBFetch(&sbas_mb[sbas_sid_to_index(nav_msg->sid)],
                    (msg_t *)&bit, TIME_IMMEDIATE) == RDY_OK) {
        nav_msg->symbols[nav_msg->symbol_count++] = bit;
        if (nav_msg->symbol_count == SBAS_NAVFLEN) {
          ephemeris_xyz_t e; e.toe.wn = WN_UNKNOWN;
          sbas_almanac_t alm[3];

          memset(&e, 0, sizeof(ephemeris_xyz_t));
          for (u8 j = 0; j < 3; j++)
            memset(&alm[j], 0, sizeof(sbas_almanac_t));

          /*
           *Decide nav msg polarity based on # of runs of decoder and how many
           *normal or inverse msgs we got.
           */
          if (nav_msg->dec_passes > 1 && nav_msg->msg_normal == 0 &&
              nav_msg->polarity == 0) {
            nav_msg->bit_polarity = BIT_POLARITY_INVERTED;
            nav_msg->polarity = 1;
          }
          else if (nav_msg->dec_passes > 1 && nav_msg->msg_inverse == 0 &&
                   nav_msg->polarity == 1) {
            nav_msg->bit_polarity = BIT_POLARITY_NORMAL;
            nav_msg->polarity = 0;
          }

          /*
           *Decode based on polarity of the singal.
           */
          init_viterbi27(&decoder, 0);
          if (nav_msg->polarity == 0) {
            update_viterbi27_blk(&decoder, nav_msg->symbols, SBAS_BITS_UPDATE);
          } else {
            update_viterbi27_blk(&decoder, nav_msg->symbols + 1,
                                 SBAS_BITS_UPDATE);
          }
          chainback_viterbi27(&decoder, nav_msg->decoded,
                              SBAS_BITS_CHAINBACK, 0);

          if (nav_msg->polarity == 0) {
            nav_msg->bit_polarity = BIT_POLARITY_NORMAL;
          }
          else {
            nav_msg->bit_polarity = BIT_POLARITY_INVERTED;
          }

          nav_msg->dec_passes++;
          if (nav_msg->dec_passes > 100) {
            nav_msg->dec_passes++;
            nav_msg->dec_passes %= 50;
          }

          /*
           *Try to see if we have valid msgs.
           */
          if (
             (time_quality == TIME_FINE || time_quality == TIME_COARSE) &&
              l1_sbas_process_subframe(nav_msg, &e, alm)) {
            if (e.valid == 1) {
              e.sid = nav_msg->sid;
              ephemeris_t eph;
              eph.ephemeris_xyz = &e;
              e.toe = position_solution.time;

              /*
               *Set TOW into the tracking channel.
               */
              for (u8 n = 0; n < MAX_CHANNELS; n++) {
                tracking_channel_t* chan = &tracking_channel[n];
                if (chan->sid.prn == e.sid.prn) {
                  chan->TOW_ms = position_solution.time.tow;
                }
              }

              ephemeris_new(&eph, nav_msg->sid);

              if (!e.healthy) {
                log_info("PRN %d unhealthy!", e.sid.prn + 1);
              } else {
                msg_ephemeris_xyz_t msg;
                pack_ephemeris_xyz(&e, &msg);
                sbp_send_msg(SBP_MSG_EPHEMERIS_XYZ,
                             sizeof(msg_ephemeris_xyz_t), (u8 *)&msg);
              }
            }

            if (alm[0].valid == 1) {
              for (u8 j = 0; j < WAAS_SATS; j++) {
                /*
                 *Be sure that almanac can be used.
                 */
                if (alm[j].sid.prn != 0) {
                  signal_t sid; sid = alm[j].sid;
                  memcpy(&sbas_almanac[sbas_sid_to_index(sid)],
                         &alm[j], sizeof(sbas_almanac_t));
                }
              }
            }
          }

          /*
           *Save last 12 bits for next decoding round.
           */
          for(u8 j = 0; j < BITS_TO_KEEP; j++)
            nav_msg->symbols[j] = nav_msg->symbols[SBAS_NAVFLEN + j -
                                                   BITS_TO_KEEP];

          nav_msg->symbol_count = BITS_TO_KEEP;
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

