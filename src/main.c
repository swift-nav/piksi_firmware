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
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "cw.h"
#include "sbp.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"
#include "track.h"
#include "acq.h"
#include "nmea.h"
#include "rtcm.h"
#include "manage.h"
#include "timing.h"
#include "position.h"
#include "peripherals/spi.h"
#include "board/leds.h"
#include "board/m25_flash.h"

#include <libswiftnav/pvt.h>
#include <libswiftnav/sbp.h>
#include <libswiftnav/track.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include "settings.h"

#define MAX_SATS 14

channel_measurement_t meas[MAX_SATS];
channel_measurement_t meas_old[MAX_SATS];
u8 n_obs;
gps_time_t t_obs;
navigation_measurement_t nav_meas[MAX_SATS];
navigation_measurement_t nav_meas_old[MAX_SATS];
rtcm_t rtcm;

void sendrtcmnav(ephemeris_t *eph, u8 prn)
{
  memset(&rtcm, 0, sizeof(rtcm));

	rtcm.eph = eph;
  rtcm.prn = prn;
  gen_rtcm3(&rtcm,1019,0);

  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if (settings.ftdi_usart.mode == RTCM)
    usart_write_dma(&ftdi_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uarta_usart.mode == RTCM)
    usart_write_dma(&uarta_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uartb_usart.mode == RTCM)
    usart_write_dma(&uartb_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  __asm__("CPSIE i;");  /* Re-enable interrupts. */
}

void sendrtcmobs(navigation_measurement_t *obs, int nsat, gps_time_t t)
{
  memset(&rtcm, 0, sizeof(rtcm));

	/* observation */
	rtcm.time=t;
	rtcm.n=nsat;
	memcpy(rtcm.obs, obs, sizeof(navigation_measurement_t)*nsat);

	/* GPS observations */
	gen_rtcm3(&rtcm,1002,0);

  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if (settings.ftdi_usart.mode == RTCM)
    usart_write_dma(&ftdi_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uarta_usart.mode == RTCM)
    usart_write_dma(&uarta_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  if (settings.uartb_usart.mode == RTCM)
    usart_write_dma(&uartb_tx_state, (u8 *)rtcm.buff, rtcm.nbyte);

  __asm__("CPSIE i;");  /* Re-enable interrupts. */
}

int _getpid()
{
  return 1;
}
void _exit(int rc)
{
  (void)rc;
  while(1);
}
int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  return -1; /* Always fails */
}


u8 rx_obs_n = 0;
msg_obs_hdr_t rx_obs_hdr;
navigation_measurement_t rx_nav_meas[MAX_SATS];

u8 reset_ambs = 0;

int nav_meas_cmp(const void *a, const void *b)
{
  return (s8)((navigation_measurement_t*)a)->prn
       - (s8)((navigation_measurement_t*)b)->prn;
}

typedef struct {
  float snr;
  u8 ia, ib;
  u8 prn;
} cmn_prn_t;

int cmn_prn_cmp_snr(const void *a, const void *b)
{
  return (((cmn_prn_t*)a)->snr > ((cmn_prn_t*)b)->snr)
       - (((cmn_prn_t*)a)->snr < ((cmn_prn_t*)b)->snr);
}

typedef struct {
  s32 N;
  u8 prn;
} amb_t;

typedef struct {
  u8 n;
  u8 ref;
  amb_t ambs[MAX_SATS];
} ambiguity_set_t;

/* ASSUMES ma, mb SORTED BY PRN. */
/* Returns prns sorted by prn. */
u8 select_sats(u8 na, navigation_measurement_t* ma,
               u8 nb, navigation_measurement_t* mb,
               cmn_prn_t prns[MAX_SATS])
{
  u8 i, j, n = 0;

  for (i=0, j=0; i<na && j<nb; i++, j++) {
    if (ma[i].prn < mb[j].prn)
      j--;
    else if (ma[i].prn > mb[j].prn)
      i--;
    else {
      prns[n].prn = ma[i].prn;
      prns[n].snr = ma[i].snr;
      prns[n].ia = i;
      prns[n].ib = j;
      n++;
    }
  }

  return n;
}

int amb_search_prn(const void *a, const void *b)
{
  return (*(u8*)a > ((amb_t*)b)->prn)
       - (*(u8*)a < ((amb_t*)b)->prn);
}

int amb_cmp_prn(const void *a, const void *b)
{
  return (((amb_t*)a)->prn > ((amb_t*)b)->prn)
       - (((amb_t*)a)->prn < ((amb_t*)b)->prn);
}

double cp_sd(gps_time_t ta, navigation_measurement_t* ma,
             gps_time_t tb, navigation_measurement_t* mb)
{
  double dt = gpsdifftime(ta, tb);
  return ma->carrier_phase - (mb->carrier_phase + dt*mb->doppler);
}

s8 change_ref(ambiguity_set_t *amb_set, u8 new_ref)
{
  if (amb_set->n == 0)
    return -1;

  if (new_ref == amb_set->ref)
    return 0;

  u8 i;

  /* TODO: bsearch here for faster lookup. */
  for (i=0; i<amb_set->n; i++) {
    if (amb_set->ambs[i].prn == new_ref) {
      break;
    }
  }

  if (i >= amb_set->n)
    /* Requested reference not in ambiguity set */
    return -1;

  s32 old2new = amb_set->ambs[i].N;
  for (u8 j=0; j<amb_set->n; j++) {
    /* N - new_ref = (N - old_ref) - (new_ref - old_ref) */
    amb_set->ambs[j].N -= old2new;
  }
  /*amb_set->ambs[i].N = -old2new;*/
  amb_set->ref = new_ref;

  return 0;
}


void dd_soln(gps_time_t ta, u8 na, navigation_measurement_t* ma,
             gps_time_t tb, u8 nb, navigation_measurement_t* mb)
{
  static ambiguity_set_t amb_set = { .n = 0, .ref = -1 };

  /* Sort observations by PRN. */
  qsort(ma, na, sizeof(navigation_measurement_t), nav_meas_cmp);
  qsort(mb, nb, sizeof(navigation_measurement_t), nav_meas_cmp);

  /*
  printf("A: ");
  for (u8 i=0; i<na; i++)
    printf("%d  ", ma[i].prn);
  printf("\n");

  printf("B: ");
  for (u8 i=0; i<nb; i++)
    printf("%d  ", mb[i].prn);
  printf("\n");
  */

  cmn_prn_t prns[MAX_SATS];
  cmn_prn_t prns_with_ambs[MAX_SATS];
  cmn_prn_t prns_without_ambs[MAX_SATS];

  u8 n_with = 0;
  u8 n_without = 0;

  if (gpsdifftime(ta, tb)*1e3 > 20) {
    printf("DD dt too great (%.2f)\n", gpsdifftime(ta, tb)*1e3);
    return;
  }

  /* Find common satellites. */
  u8 n_common = select_sats(na, ma, nb, mb, prns);

  if (n_common < 4) {
    /*printf("Too few common sats for DD soln (%d)\n", n_common);*/
    return;
  }

  if (reset_ambs) {
    reset_ambs = 0;
    printf("Reset Ambiguities! %d\n", n_common);

    /* Find PRN with max SNR. */
    u8 ref_idx = 0;
    float max_snr = 0;
    for (u8 i=0; i<n_common; i++) {
      if (prns[i].snr > max_snr)
        ref_idx = i;
    }

    amb_set.n = n_common;
    amb_set.ref = prns[ref_idx].prn;

    double ref_sd = cp_sd(ta, &ma[prns[ref_idx].ia], tb, &mb[prns[ref_idx].ib]);
    printf("RESET DDs dt: %.2fms, ref: %02d\n", gpsdifftime(ta, tb)*1e3, (int)prns_with_ambs[ref_idx].prn+1);
    for (u8 i=0; i<n_common; i++) {
      double dd = cp_sd(ta, &ma[prns[i].ia], tb, &mb[prns[i].ib]) - ref_sd;
      amb_set.ambs[i].N = (s32)round(dd);
      amb_set.ambs[i].prn = prns[i].prn;
    }
    return;
  }

  amb_t *amb;
  amb_t ambs[MAX_SATS];

  /* Separate into those with resolved ambiguities and those without. */
  for (u8 i=0; i<n_common; i++) {
    /* TODO: integrate this into select_sats. Should be able to integrate it
     * into the same for loop and do everything in O(n) if ambs are also
     * pre-sorted by PRN. */
    if ((amb = bsearch(&(prns[i].prn), amb_set.ambs, amb_set.n, sizeof(amb_t), amb_search_prn))) {
      prns_with_ambs[n_with] = prns[i];
      /* Make list of ambs with the same indicies as prns_with_ambs. */
      ambs[n_with] = *amb;
      n_with++;
    } else {
      prns_without_ambs[n_without] = prns[i];
      n_without++;
    }
  }

  /*
  printf("Unresolved: %d\n", n_without);
  for (u8 i=0; i<n_without; i++)
    printf("  %d\t%d\t%d\n", prns_without_ambs[i].prn+1, prns_without_ambs[i].ia, prns_without_ambs[i].ib);

  printf("Resolved: %d\n", n_with);
  for (u8 i=0; i<n_with; i++)
    printf("  %d\t%d\t%d\n", prns_with_ambs[i].prn+1, prns_with_ambs[i].ia, prns_with_ambs[i].ib);
  */
  double b[3];
  double dds[MAX_SATS];

  if (n_with >= 4) {

    /* Select reference satellite from the prns with resolved ambigities.
     * For now just use sat with highest SNR. */
    /* TODO: Add hysteresis? */
    u8 ref_idx = 0;
    float max_snr = 0;
    for (u8 i=0; i<n_with; i++) {
      /* TODO: Don't need to store SNR in prns now really. */
      if (prns_with_ambs[i].snr > max_snr) {
        max_snr = prns_with_ambs[i].snr;
        ref_idx = i;
      }
    }

    if (prns_with_ambs[ref_idx].prn != amb_set.ref) {
      /* TODO: Could remove checks from change_ref for prn not in ambs? Probably best to leave it in. */
      printf("Reference sat changed %d -> %d\n", amb_set.ref, prns_with_ambs[ref_idx].prn);
      /* TODO: Need to update the local var 'ambs' ie. the ambs list indexed with prns_with_ambs. */
      s8 ret = change_ref(&amb_set, prns_with_ambs[ref_idx].prn);
      if (ret < 0)
        printf("Error in change_ref\n");
    }

    double ref_sd = cp_sd(ta, &ma[prns_with_ambs[ref_idx].ia], tb, &mb[prns_with_ambs[ref_idx].ib]);
    for (u8 i=0; i<n_with; i++) {
      dds[i] = cp_sd(ta, &ma[prns_with_ambs[i].ia], tb, &mb[prns_with_ambs[i].ib]) - ref_sd;
      dds[i] -= ambs[i].N;
    }
    DO_EVERY(10,
    printf("DDs dt: %.2fms, ref: %02d\n", gpsdifftime(ta, tb)*1e3, (int)prns_with_ambs[ref_idx].prn+1);
    for (u8 i=0; i<n_with; i++) {
      printf("  %.3f\n", dds[i]*0.19029);
    }
    );

    /* Construct DD geometry matrix G. */
    double ref_los[3]; /* Line of sight vector to reference satellite. */
    double los[3]; /* Line of sight vector to satellite. */
    double G[14][3];
    double temp[14];

    vector_subtract(3, position_solution.pos_ecef, ma[prns_with_ambs[ref_idx].ia].sat_pos, ref_los);
    vector_normalize(3, ref_los);

    u8 j = 0;
    for (u8 i=0; i<n_with; i++) {
      vector_subtract(3, position_solution.pos_ecef, ma[prns_with_ambs[i].ia].sat_pos, los);
      vector_normalize(3, los);
      if (i != ref_idx) {
        vector_subtract(3, los, ref_los, G[j]);
        temp[j] = dds[i];
        j++;
      }
    }

    double Gtrans[3][14];
    double GtG[3][3];
    double H[3][3];
    double X[3][14];

    /* Gt := G^{T} */
    matrix_transpose(n_with-1, 3, (double *) G, (double *) Gtrans);
    /* GtG := G^{T} G */
    matrix_multiply(3, n_with-1, 3, (double *) Gtrans, (double *) G, (double *) GtG);
    /* H \elem \mathbb{R}^{3 \times 3} := GtG^{-1} */
    matrix_inverse(3, (const double *) GtG, (double *) H);
    /* X := H * G^{T} */
    matrix_multiply(3, 3, n_with-1, (double *) H, (double *) Gtrans, (double *) X);
    /* baseline := X * DD */
    matrix_multiply(3, n_with-1, 1, (double *) X, (double *) temp, (double *) b);

    for (u8 i=0; i<3; i++)
      b[i] = b[i] * 0.190293673;


    msg_baseline_t bsln;
    bsln.n_sats = n_with;
    bsln.flags = 0;
    bsln.t = ta;
    wgsecef2ned(b, position_solution.pos_ecef, bsln.ned);
    sbp_send_msg(MSG_BASELINE, sizeof(msg_baseline_t), (u8 *)&bsln);

    if (n_without > 0) {
      /* Now initialise new sats using the computed baseline. */
      double dlos[3];
      double dd, meas_dd;
      for (u8 i=0; i<n_without; i++) {
        /* First compute expected double diff
         *  DD = b . (los - ref_los) */
        vector_subtract(3, position_solution.pos_ecef, ma[prns_without_ambs[i].ia].sat_pos, los);
        vector_normalize(3, los);
        vector_subtract(3, los, ref_los, dlos);
        dd = vector_dot(3, b, dlos) / 0.190293673;
        /* Now compute measured DD. */
        meas_dd = cp_sd(ta, &ma[prns_without_ambs[i].ia], tb, &mb[prns_without_ambs[i].ib]) - ref_sd;
        /* Ambiguity is the difference of the two.
         * DD = meas_dd - amb
         *  => amb = meas_dd - DD */
        amb_set.ambs[amb_set.n].N = (s32)round(meas_dd - dd);
        double res = meas_dd - dd - amb_set.ambs[amb_set.n].N;
        printf("Resolving %d - residual %.3f\n", prns_without_ambs[i].prn, res*0.190293673);
        amb_set.ambs[amb_set.n].prn = prns_without_ambs[i].prn;
        amb_set.n++;
      }
      /* Sort ambs to maintain prn order. */
      qsort(amb_set.ambs, amb_set.n, sizeof(amb_t), amb_cmp_prn);
    }
  }
}

void foo_callback(u16 sender_id, u8 len, u8 buff[])
{
  (void)sender_id; (void)len;

  settings.uarta_usart.message_mask = buff[0];
}

void reset_ambs_callback(u16 sender_id, u8 len, u8 buff[])
{
  (void)sender_id; (void)len; (void)buff;
  reset_ambs = 1;
}
void obs_hdr_callback(u16 sender_id, u8 len, u8 buff[])
{
  (void)sender_id; (void)len;
  rx_obs_n = 0;
  rx_obs_hdr = *(msg_obs_hdr_t*)buff;
}
void obs_callback(u16 sender_id, u8 len, u8 buff[])
{
  (void)sender_id; (void)len;
  msg_obs_t *obs = (msg_obs_t *)buff;
  /* TODO: use msg_obs_t directly. */
  rx_nav_meas[rx_obs_n].prn = obs->prn;
  rx_nav_meas[rx_obs_n].raw_pseudorange = obs->P;
  rx_nav_meas[rx_obs_n].carrier_phase = obs->L;
  rx_nav_meas[rx_obs_n].doppler = obs->D;
  rx_nav_meas[rx_obs_n].snr = obs->snr;
  rx_obs_n++;
  if (rx_obs_n == rx_obs_hdr.n_obs) {
    /* Process DD solution. */
    dd_soln(t_obs, n_obs, nav_meas,
            rx_obs_hdr.t, rx_obs_n, rx_nav_meas);
  }
}

void drop_chan_callback(u16 sender_id, u8 len, u8 buff[])
{
  (void)sender_id; (void)len;
  tracking_channel_disable(buff[0]);
}

void send_observations(u8 n, navigation_measurement_t *m)
{
  msg_obs_t obs;
  for (u8 i=0; i<n; i++) {
    obs.prn = m[i].prn;
    obs.P = m[i].raw_pseudorange;
    obs.L = m[i].carrier_phase;
    obs.D = m[i].doppler;
    obs.snr = m[i].snr;
    obs.lock_count = 255;
    obs.flags = 0;
    obs.obs_n = i;
    sbp_send_msg(MSG_OBS, sizeof(obs), (u8 *)&obs);
  }
}

int main(void)
{
  init(1);

  led_toggle(LED_RED);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  u8 nap_git_hash[20];
  nap_conf_rd_git_hash(nap_git_hash);
  printf("SwiftNAP git: ");
  for (u8 i=0; i<20; i++)
    printf("%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    printf(" (unclean)");
  printf("\n");
  printf("SwiftNAP configured with %d tracking channels\n\n", nap_track_n_channels);

  /*helloWorld();*/

  cw_setup();
  manage_acq_setup();
  tick_timer_setup();
  timing_setup();
  position_setup();

  static sbp_msg_callbacks_node_t obs_hdr_node;
  static sbp_msg_callbacks_node_t obs_node;
  static sbp_msg_callbacks_node_t reset_ambs_node;
  static sbp_msg_callbacks_node_t foo_node;
  static sbp_msg_callbacks_node_t drop_chan_node;

  sbp_register_callback(MSG_OBS_HDR, &obs_hdr_callback, &obs_hdr_node);
  sbp_register_callback(MSG_OBS, &obs_callback, &obs_node);
  sbp_register_callback(0x99, &reset_ambs_callback, &reset_ambs_node);
  sbp_register_callback(0x98, &foo_callback, &foo_node);
  sbp_register_callback(0x96, &drop_chan_callback, &drop_chan_node);

  /* TODO: Think about thread safety when updating ephemerides. */
  static ephemeris_t es[32];
  static ephemeris_t es_old[32];
  while(1)
  {
    sbp_process_messages();
    manage_track();
    manage_acq();

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    memcpy(es_old, es, sizeof(es));
    for (u8 i=0; i<nap_track_n_channels; i++)
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        s8 ret = process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
        if (ret < 0)
          printf("PRN %02d ret %d\n", tracking_channel[i].prn+1, ret);

        if (ret == 1 && !es[tracking_channel[i].prn].healthy)
          printf("PRN %02d unhealthy\n", tracking_channel[i].prn+1);
        if (memcmp(&es[tracking_channel[i].prn], &es_old[tracking_channel[i].prn], sizeof(ephemeris_t))) {
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
        if (es[tracking_channel[i].prn].valid == 1) {
          sendrtcmnav(&es[tracking_channel[i].prn], tracking_channel[i].prn);
        }
      }

    static u32 last_tow = 0;
    if (tracking_channel[0].state == TRACKING_RUNNING &&
        (tracking_channel[0].TOW_ms - last_tow > 100) &&
        ((tracking_channel[0].TOW_ms + 70) % 200 < 50))
    {
    last_tow = tracking_channel[0].TOW_ms;

      u8 n_ready = 0;
      for (u8 i=0; i<nap_track_n_channels; i++) {
        if (es[tracking_channel[i].prn].valid == 1 && \
            es[tracking_channel[i].prn].healthy == 1 && \
            tracking_channel[i].state == TRACKING_RUNNING && \
            tracking_channel[i].TOW_ms > 0) {
          memcpy(meas_old, meas, sizeof(meas));
          __asm__("CPSID i;");
          tracking_update_measurement(i, &meas[n_ready]);
          __asm__("CPSIE i;");

          if (meas[n_ready].snr > 2)
            n_ready++;
        }
      }

      if (n_ready >= 4) {
        /* Got enough sats/ephemerides, do a solution. */
        /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
         * more intelligent with the solution time.
         */
        u64 nav_tc = nap_timing_count();
        memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
        calc_navigation_measurement(n_ready, meas, nav_meas, (double)((u32)nav_tc)/SAMPLE_FREQ, es);

        dops_t dops;
        if (calc_PVT(n_ready, nav_meas, &position_solution, &dops) == 0) {
          position_updated();

          n_obs = n_ready;
          t_obs = position_solution.time;
/*
          set_time_fine(nav_tc, position_solution.clock_bias, position_solution.time);
          printf("dt: %g\n", gpsdifftime(position_solution.time, rx2gpstime(nav_tc)));
          printf("est clock freq: %g\n", 16.368e6 - 1.0/clock_state.clock_period);
          printf("clock offset: %g, clock bias: %g\n", position_solution.clock_offset, position_solution.clock_bias);
*/
          static u8 obs_count = 0;
          msg_obs_hdr_t obs_hdr = { .t = position_solution.time, .count = obs_count, .n_obs = n_ready };
          sbp_send_msg(MSG_OBS_HDR, sizeof(obs_hdr), (u8 *)&obs_hdr);
          send_observations(n_ready, nav_meas);
          obs_count++;

          sbp_send_msg(MSG_SOLUTION, sizeof(gnss_solution), (u8 *) &position_solution);
          nmea_gpgga(&position_solution, &dops);

          sendrtcmobs(nav_meas, n_ready, position_solution.time);

          DO_EVERY(10,
            sbp_send_msg(MSG_DOPS, sizeof(dops_t), (u8 *) &dops);
            nmea_gpgsv(n_ready, nav_meas, &position_solution);
          );
        }
      }
    }

    DO_EVERY_TICKS(TICK_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_TICKS(TICK_FREQ/5, // 5 Hz update
      tracking_send_state();
    );

    u32 err = nap_error_rd_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}





