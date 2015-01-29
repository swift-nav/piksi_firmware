/*
 * Copyright (C) 2014 Swift Navigation Inc.
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
#include <stdlib.h>
#include <math.h>

#include <libswiftnav/sbp_utils.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include "board/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "solution.h"
#include "manage.h"
#include "simulator.h"
#include "settings.h"
#include "timing.h"
#include "base_obs.h"

Mutex base_obs_lock;
BinarySemaphore base_obs_received;

extern ephemeris_t es[MAX_SATS];
obss_t base_obss;

Mutex base_pos_lock;
bool_t base_pos_known = false;
double base_pos_ecef[3];

void base_pos_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) msg; (void) sender_id;

  double llh_degrees[3];
  double llh[3];
  memcpy(llh_degrees, msg, 3*sizeof(double));

  llh[0] = llh_degrees[0] * D2R;
  llh[1] = llh_degrees[1] * D2R;
  llh[2] = llh_degrees[2];

  chMtxLock(&base_pos_lock);
  wgsllh2ecef(llh, base_pos_ecef);
  base_pos_known = true;
  chMtxUnlock();
}


void obs_old_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;
  (void) len;
  (void) msg;
  (void) sender_id;

  printf("Receiving an old deprecated observation message.\n");
  printf("Please update your base station firmware.\n");
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  static s16 prev_count = 0;
  static gps_time_t prev_t = {.tow = 0.0, .wn = 0};
  /* Using an extra obss_t so we don't need to overwrite the old set (used for
   * low latency) when we end up with a bad new set. */
  static obss_t base_obss_raw = {.has_pos = 0};

  /* Sender ID of zero means that the messages are relayed observations,
   * ignore them. */
  if (sender_id == 0)
    return;

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(MSG_PACKED_OBS, len, msg, 0);

  gps_time_t t;
  u8 total;
  u8 count;
  unpack_obs_header((msg_obs_header_t*)msg, &t, &total, &count);
  double epoch_count = t.tow * (soln_freq / obs_output_divisor);

  if (fabs(epoch_count - round(epoch_count)) > TIME_MATCH_THRESHOLD) {
    printf("Unaligned observation from base station ignored.\n");
    return;
  }

  /* Calculate packet latency */
  if (time_quality >= TIME_COARSE) {
    gps_time_t now = get_current_time();
    float latency_ms = (float) ((now.tow - t.tow) * 1000.0);

    log_obs_latency(latency_ms);
  }

  /* Verify sequence integrity */
  if (count == 0) {
    prev_t = t;
    prev_count = 0;
  } else if (prev_t.tow != t.tow ||
        prev_t.wn != t.wn ||
        prev_count + 1 != count) {
      printf("Dropped one of the observation packets! Skipping this sequence.\n");
      prev_count = -1;
      return;
  } else {
      prev_count = count;
  }

  u8 obs_in_msg = (len - sizeof(msg_obs_header_t)) / sizeof(msg_obs_content_t);

  if (count == 0) {
    base_obss_raw.n = 0;
    base_obss_raw.t = t;
  }
  msg_obs_content_t *obs = (msg_obs_content_t *)(msg + sizeof(msg_obs_header_t));
  for (u8 i=0; i<obs_in_msg; i++) {
    if (ephemeris_good(es[obs[i].prn], t)) {
      unpack_obs_content(&obs[i],
        &base_obss_raw.nm[base_obss_raw.n].raw_pseudorange,
        &base_obss_raw.nm[base_obss_raw.n].carrier_phase,
        &base_obss_raw.nm[base_obss_raw.n].snr,
        &base_obss_raw.nm[base_obss_raw.n].lock_counter,
        &base_obss_raw.nm[base_obss_raw.n].prn);
      double clock_err;
      double clock_rate_err;
      calc_sat_pos(base_obss_raw.nm[base_obss_raw.n].sat_pos,
                   base_obss_raw.nm[base_obss_raw.n].sat_vel,
                   &clock_err, &clock_rate_err, &es[obs[i].prn], t);
      /* TODO Make a function to apply some of these corrections.
       *       They are used in a couple places. */
      base_obss_raw.nm[base_obss_raw.n].pseudorange =
            base_obss_raw.nm[base_obss_raw.n].raw_pseudorange + clock_err * GPS_C;
      base_obss_raw.nm[base_obss_raw.n].tot = t;
      /* set the time */
      base_obss_raw.n++;
    }
  }

  /* If we can, and all the obs have been received, calculate the receiver
   * position. */
  if (count == total - 1) {
    /* Ensure observations sorted by PRN. */
    qsort(base_obss_raw.nm, base_obss_raw.n,
          sizeof(navigation_measurement_t), nav_meas_cmp);

    /* Lock mutex before modifying base_obss.
     * NOTE: We didn't need to lock it before reading in THIS context as this
     * is the only thread that writes to base_obss. */
    chMtxLock(&base_obs_lock);

    /* Create a set of navigation measurements to store the previous
     * observations. */
    static u8 n_old = 0;
    static navigation_measurement_t nm_old[MAX_CHANNELS];

    /* Fill in the navigation measurements in base_obss, using TDCP method to
     * calculate the Doppler shift. */
    base_obss.n = tdcp_doppler(base_obss_raw.n, base_obss_raw.nm,
                               n_old, nm_old, base_obss.nm);
    base_obss.t = base_obss_raw.t;

    /* Copy the current observations over to nm_old so we can difference
     * against them next time around. */
    memcpy(nm_old, base_obss_raw.nm,
           base_obss_raw.n * sizeof(navigation_measurement_t));
    n_old = base_obss_raw.n;

    if (base_obss.n >= 4) {
      /* No need to lock before reading here as base_pos_* is only written from
       * this thread (SBP). */
      if (base_pos_known) {
        memcpy(base_obss.pos_ecef, base_pos_ecef, 3 * sizeof(double));
        base_obss.has_pos = 1;
      } else {
        gnss_solution soln;
        dops_t dops;

        s32 ret = calc_PVT(base_obss.n, base_obss.nm, &soln, &dops);

        if (ret == 0 && soln.valid) {
          if (base_obss.has_pos) {
            /* TODO Implement a real filter for base position (potentially in
               observation space), so we can do away with this terrible excuse
               for smoothing. */
            base_obss.pos_ecef[0] = 0.99995 * base_obss.pos_ecef[0]
                                      + 0.00005 * soln.pos_ecef[0];
            base_obss.pos_ecef[1] = 0.99995 * base_obss.pos_ecef[1]
                                      + 0.00005 * soln.pos_ecef[1];
            base_obss.pos_ecef[2] = 0.99995 * base_obss.pos_ecef[2]
                                      + 0.00005 * soln.pos_ecef[2];
          } else {
            memcpy(base_obss.pos_ecef, soln.pos_ecef, 3 * sizeof(double));
          }
          base_obss.has_pos = 1;
        } else {
          printf("Error calculating base station position (%ld)\n", ret);
          if (base_obss.has_pos) {
            memcpy(base_obss.pos_ecef, base_obss.pos_ecef, 3 * sizeof(double));
            base_obss.has_pos = 1;
          } else {
            base_obss.has_pos = 0;
          }
        }
      }

      if (base_obss.has_pos) {
        for (u8 i=0; i < base_obss.n; i++) {
          double dx[3];
          vector_subtract(3, base_obss.nm[i].sat_pos, base_obss.pos_ecef, dx);
          base_obss.sat_dists[i] = vector_norm(3, dx);
        }
      }
    }
    /* Unlock base_obss mutex. */
    chMtxUnlock();
    /* Signal that a complete base observation has been received. */
    chBSemSignal(&base_obs_received);
  }
}

void base_obs_setup()
{
  chMtxInit(&base_obs_lock);
  chBSemInit(&base_obs_received, TRUE);
  chMtxInit(&base_pos_lock);

  static sbp_msg_callbacks_node_t base_pos_node;
  sbp_register_cbk(
    MSG_BASE_POS,
    &base_pos_callback,
    &base_pos_node
  );

  static sbp_msg_callbacks_node_t obs_old_node;
  sbp_register_cbk(
    MSG_OLD_OBS,
    &obs_old_callback,
    &obs_old_node
  );

  static sbp_msg_callbacks_node_t obs_packed_node;
  sbp_register_cbk(
    MSG_PACKED_OBS,
    &obs_callback,
    &obs_packed_node
  );
}


