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
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

struct v27 decoder;
decision_t decisions[DECISION_SIZE] _CCM;
extern tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];
extern sbas_nav_msg_t sbas_nav_msgs[WAAS_SATS];

const u8 sbas_preambles[] = {0x53, 0x9A, 0xC6};
const u8 sbas_preambles_inv[] = {0xAC, 0x65, 0x39};

MUTEX_DECL(decoder_mtx);
/*
 *
 *static bool decoder_find_preamble(unsigned char p, bool *stat)
 *{
 *  for (u8 i = 0; i < 3; i++) {
 *    if (p == sbas_preambles[i]) {
 *      *stat = true;
 *      return true;
 *    }
 *    else if (p == sbas_preambles_inv[i]) {
 *      *stat = false;
 *      return true;
 *    }
 *  }
 *  return false;
 *}
 *
 *static u8 decoder_flip_bits(u8 value)
 *{
 *  return ~value;
 *}
 */

static WORKING_AREA_CCM(wa_decoder_thread, 1000);
static msg_t decoder_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decoder");
  while (TRUE) {
    watchdog_notify(WD_NOTIFY_DECODER);
    for (u8 i = 0; i < WAAS_SATS; i++) {
      sbas_nav_msg_t *nav_msg = &sbas_nav_msgs[i];
      /*
       *signal_t *sid = &nav_msg->sid;
       */

      chMtxLock(&decoder_mtx);

      if (nav_msg->symbol_count == SBAS_NAVFLEN) {
        /*
         *if (SBAS_DEC_SIZE != cfs_write(fd, nav_msg->symbols, SBAS_NAVFLEN)) {
         *  log_info("File is full!");
         *}
         */
/*
 *         init_viterbi27(&decoder, 0);
 *        update_viterbi27_blk(&decoder, nav_msg->symbols,
 *                             SBAS_BITS_UPDATE);
 *        chainback_viterbi27(&decoder, nav_msg->decoded,
 *                            SBAS_BITS_CHAINBACK, 0);
 *
 *
 *        for (u8 j = 0; j < BITS_TO_KEEP; j++)
 *          nav_msg->symbols[j] = nav_msg->symbols[SBAS_NAVFLEN -
 *                                                 BITS_TO_KEEP +
 *                                                 j];
 *
 *        u8 bits_inverse[SBAS_DEC_SIZE];
 *        for (u16 j = 0; j < SBAS_DEC_SIZE; j++)
 *          bits_inverse[j] = decoder_flip_bits(nav_msg->decoded[j]);
 *
 *        for (u16 j = 0; j < SBAS_DEC_SIZE; j++) {
 *          bool type = false;
 *          bool ret = false;
 *
 *          ret = decoder_find_preamble(nav_msg->decoded[j], &type);
 *          if (ret) {
 *            unsigned char *buf;
 *            if (type) {
 *              buf = nav_msg->decoded;
 *            } else {
 *              buf = bits_inverse;
 *            }
 *            u32 msg_type = getbitu(buf+ j + 1, 0, 6);
 *            if (msg_type != 9) {
 *              log_info("Msg type: %d for PRN %d", msg_type, sid->prn + 1);
 *            } else {
 *              time_t now = gps2time(get_current_time());
 *              log_info("Msg type: %d for PRN %d @ %d", msg_type, sid->prn +1, now);
 *            }
 *          }
 *        }
 */
        nav_msg->symbol_count = 0;
      }
      chMtxUnlock();
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

