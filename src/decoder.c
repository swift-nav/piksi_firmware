/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Vlad Ungureanu <vvu@vdev.ro>
 *          Dennis Zollo <dennis@swift-nav.com>
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
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

struct v27 decoder _CCM;
decision_t decisions[DECISION_SIZE] _CCM;

extern time_quality_t time_quality;
extern sbas_almanac_t sbas_almanac[WAAS_SATS];
extern gnss_solution position_solution;
extern l1_sbas_nav_msg_t sbas_nav_msgs[WAAS_SATS];
extern Mailbox sbas_mb[3];
extern Mutex track_mutex;

static WORKING_AREA_CCM(wa_decoder_thread, 1500);
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

      u8 bit = 0;
      u32 full_info = 0;
      if (chMBFetch(&sbas_mb[sbas_sid_to_index(nav_msg->sid)],
                    (msg_t *)&full_info, TIME_IMMEDIATE) == RDY_OK) {
        if (full_info&1)
          bit = 255;
        /*
         *Channel time in ms when the last bit of the decoder frame is received
         */
        u32 chan_time = full_info >> 1;

        nav_msg->symbols[nav_msg->symbol_count++] = bit;
        if (nav_msg->symbol_count == SBAS_NAVFLEN) {
         ephemeris_xyz_t e; e.toe.wn = WN_UNKNOWN;
         sbas_almanac_t alm[3];

          memset(&e, 0xFF, sizeof(ephemeris_xyz_t));
          for (u8 j = 0; j < 3; j++)
            memset(&alm[j], 0xFF, sizeof(sbas_almanac_t));

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

          /* l1_sbas_process_subframe returns the bit offset from end of
           * buffer
           * of the preamble aligned to GPS seconds. We multiply by 2 (for
           * symbols per bit) and then by 2 again for milli seconds per symbol
           */
          s16 index = l1_sbas_process_subframe(nav_msg, &e, alm);
          s16 symbols =  2 * index;
          if (time_quality == TIME_FINE && symbols > 0 ) {

            __attribute__ ((unused)) u32 ms_offset = symbols * 2;
            __attribute__ ((unused)) u32 u_count = 0;

            /*
             *Current GPS time (SBAS network time is withing 50nS of GPS time)
             */
            u32 n_second = trunc(get_current_time().tow);
            __attribute__((unused)) s32 tow_diff = 0;

            n_second -= n_second % 3;
            u32 n_ms = n_second * 1e3;

          chMtxLock(&track_mutex);
            for (int i = 0; i < MAX_CHANNELS; i++) {
              tracking_channel_t *chan = &tracking_channel[i];
              if (chan->sid.prn == nav_msg->sid.prn) {
                u_count = chan->update_count;
               /*
                * We first take the current solution's time and round it to the
                * previous subframe boundary, and then convert to milliseconds.
                *
                * We then take the difference between the "update count" value
                * for the Last symbol in the buffer (chan_time), and our
                * current update count value from the tracking channel.  This
                * represents the delay we incurred from processing the the
                * message and is to be added to the previous GPS second.
                *
                * We then take the ms_offset parameter, which represents the
                * absolute value of the time difference between the last bit in
                * the buffer and the time at which the GPS second aligned
                * preamble was received. This value is also added to our
                * previous GPS second, as it is additional delay incurred after
                * the aligned second before the end of the buffer
                */
                tow_diff = chan->TOW_ms - u_count;
                chan->TOW_ms = n_ms + (u_count - chan_time) + ms_offset + 10;

                /*
                 *if (u_count + tow_diff != (u32)chan->TOW_ms) {
                 *  log_info("PRN %d new %d expected %d, %d", chan->sid.prn + 1,
                 *         chan->TOW_ms, u_count + tow_diff, index);
                 *}
                 */
              }
            }
          chMtxUnlock();

            if (e.valid == 1) {
              e.sid = nav_msg->sid;
              ephemeris_t eph;
              eph.ephemeris_xyz = &e;
              e.toe = get_current_time();

              /*
               *Boom, SBAS finally decided to send an ephemeris.
               */
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
              int fd = cfs_open("sbas_almanac", CFS_WRITE);
              if (fd == -1)
                log_error("Error opening almanac file");
              for (u8 j = 0; j < WAAS_SATS; j++) {
                /*
                 *Be sure that almanac can be used.
                 */
                if (alm[j].sid.prn != 0) {
                  signal_t sid; sid = alm[j].sid;
                  memcpy(&sbas_almanac[sbas_sid_to_index(sid)],
                         &alm[j], sizeof(sbas_almanac_t));
                  if (fd != -1) {
                    cfs_seek(fd, (sbas_sid_to_index(alm[j].sid))*sizeof(sbas_almanac_t), CFS_SEEK_SET);
                    if (cfs_write(fd, &alm[j], sizeof(sbas_almanac_t)) != sizeof(sbas_almanac_t)) {
                      log_error("Error writing to SBAS almanac file");
                    } else {
                      log_info("Saved PRN %d almanac to flash",
                               alm[j].sid.prn + 1);
                    }
                  }
                }
              }
              if (fd)
                cfs_close(fd);
            }
          }

          /*
           *Save last 12 symbols for next decoding round.
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


