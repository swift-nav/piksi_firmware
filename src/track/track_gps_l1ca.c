/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_gps_l1ca.h"
#include "track_gps_l2cm.h" /* for L1C/A to L2 CM tracking handover */
#include "track_api.h"
#include "track.h"
#include "decode.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>

#include "settings.h"
#include "signal.h"

/*  code: nbw zeta k carr_to_code
 carrier:                    nbw  zeta k fll_aid */
#define LOOP_PARAMS_SLOW \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5))," \
 "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))"

#define LOOP_PARAMS_MED \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5))," \
  "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))"

#define LOOP_PARAMS_FAST \
  "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5))," \
  "(4 ms, (1, 0.7, 1, 1540), (62, 0.7, 1, 0))"

#define LOOP_PARAMS_EXTRAFAST \
  "(1 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 5))," \
  "(2 ms, (1, 0.7, 1, 1540), (100, 0.7, 1, 0))"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS_PESS     "0.10, 1.4, 200, 50"
#define LD_PARAMS_NORMAL   "0.05, 1.4, 150, 50"
#define LD_PARAMS_OPT      "0.02, 1.1, 150, 50"
#define LD_PARAMS_EXTRAOPT "0.02, 0.8, 150, 50"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

#define CN0_EST_LPF_CUTOFF 5

/* Convert milliseconds to L1C/A chips */
#define L1CA_TRACK_MS_TO_CHIPS(ms) ((ms) * GPS_L1CA_CHIPS_NUM)

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  u8 coherent_ms;
} loop_params_stage[2];

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 31.0; /* dBHz */
static float track_cn0_drop_thres = 31.0;

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS_DISABLE;
static bool use_alias_detection = true;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  bool short_cycle;            /**< Set to true when a short 1ms integration is requested. */
  u8 stage;                    /**< 0 = First-stage. 1 ms integration.
                                    1 = Second-stage. After nav bit sync,
                                    retune loop filters and typically (but
                                    not necessarily) use longer integration. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
} gps_l1ca_tracker_data_t;

static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS];
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[NUM_GPS_L1CA_TRACKERS];

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);

static const tracker_interface_t tracker_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         tracker_gps_l1ca_init,
  .disable =      tracker_gps_l1ca_disable,
  .update =       tracker_gps_l1ca_update,
  .trackers =     gps_l1ca_trackers,
  .num_trackers = NUM_GPS_L1CA_TRACKERS
};

static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

void track_gps_l1ca_register(void)
{
  SETTING_NOTIFY("track", "loop_params", loop_params_string,
                 TYPE_STRING, parse_loop_params);
  SETTING_NOTIFY("track", "lock_detect_params", lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);
  SETTING("track", "cn0_use", track_cn0_use_thres, TYPE_FLOAT);
  SETTING("track", "cn0_drop", track_cn0_drop_thres, TYPE_FLOAT);
  SETTING("track", "alias_detect", use_alias_detection, TYPE_BOOL);

  for (u32 i=0; i<NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l1ca_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage[0];

  /* Note: The only coherent integration interval currently supported
     for first-stage tracking (i.e. loop_params_stage[0].coherent_ms)
     is 1. */
  data->int_ms = MIN(l->coherent_ms,
                     tracker_bit_length_get(channel_info->context));

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  data->short_cycle = true;

  /* Initialise C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3/data->int_ms, common_data->cn0, CN0_EST_LPF_CUTOFF, 1e3/data->int_ms);

  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  /* TODO: Reconfigure alias detection between stages */
  u8 alias_detect_ms = MIN(loop_params_stage[1].coherent_ms,
                           tracker_bit_length_get(channel_info->context));
  alias_detect_init(&data->alias_detect, 500/alias_detect_ms,
                    (alias_detect_ms-1)*1e-3);

}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  if ((data->int_ms > 1) && !data->short_cycle) {
    /* If we just requested the short cycle, this is the long cycle's
     * correlations. */
    corr_t cs[3];
    tracker_correlations_read(channel_info->context, cs,
                              &common_data->sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    /* accumulate short cycle correlations with long */
    for(int i = 0; i < 3; i++) {
      data->cs[i].I += cs[i].I;
      data->cs[i].Q += cs[i].Q;
    }
  } else {
    tracker_correlations_read(channel_info->context, data->cs,
                              &common_data->sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);
  }
  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                   common_data->TOW_ms,
                                   data->short_cycle ? 1 : (data->int_ms-1));

  if (data->int_ms > 1) {
    /* If we're doing long integrations, alternate between short and long
     * cycles.  This is because of FPGA pipelining and latency.  The
     * loop parameters can only be updated at the end of the second
     * integration interval and waiting a whole 20ms is too long.
     */
    data->short_cycle = !data->short_cycle;

    if (!data->short_cycle) {
      tracker_retune(channel_info->context, common_data->carrier_freq,
                     common_data->code_phase_rate,
                     L1CA_TRACK_MS_TO_CHIPS(1));
      return;
    }
  }

  common_data->update_count += data->int_ms;

  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  common_data->cn0 = cn0_est(&data->cn0_est, cs[1].I/data->int_ms, cs[1].Q/data->int_ms);
  if (common_data->cn0 > track_cn0_drop_thres)
    common_data->cn0_above_drop_thres_count = common_data->update_count;

  if (common_data->cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs[1].I, cs[1].Q, data->int_ms);
  if (data->lock_detect.outo)
    common_data->ld_opti_locked_count = common_data->update_count;
  if (!data->lock_detect.outp)
    common_data->ld_pess_unlocked_count = common_data->update_count;

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    if (data->stage > 0) {
      log_info_sid(channel_info->sid, "PLL stress");
    }
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* TODO: Make this more elegant. */
  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs[2-i].I;
    cs2[i].Q = cs[2-i].Q;
  }

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (data->int_ms > 1) {
    tracker_correlations_send(channel_info->context, cs);
  }

  aided_tl_update(&data->tl_state, cs2);
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq + GPS_CA_CHIPPING_RATE;

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (use_alias_detection &&
      (data->lock_detect.outp ||
       (data->lock_detect.outo && data->stage > 0))) {
    s32 I = (cs[1].I - data->alias_detect.first_I) / (data->int_ms - 1);
    s32 Q = (cs[1].Q - data->alias_detect.first_Q) / (data->int_ms - 1);
    float err = alias_detect_second(&data->alias_detect, I, Q);
    if (fabs(err) > (250 / data->int_ms)) {
      if (data->lock_detect.outp) {
        log_warn_sid(channel_info->sid, "False phase lock detected");
      }

      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  /* Consider moving from stage 0 (1 ms integration) to stage 1 (longer). */
  if ((data->stage == 0) &&
      /* Must have (at least optimistic) phase lock */
      (data->lock_detect.outo) &&
      /* Must have nav bit sync, and be correctly aligned */
      tracker_bit_aligned(channel_info->context)) {
    log_info_sid(channel_info->sid, "synced");
    data->stage = 1;
    const struct loop_params *l = &loop_params_stage[1];
    data->int_ms = MIN(l->coherent_ms,
                       tracker_bit_length_get(channel_info->context));
    data->short_cycle = true;

    cn0_est_init(&data->cn0_est, 1e3 / data->int_ms, common_data->cn0,
                 CN0_EST_LPF_CUTOFF, 1e3 / data->int_ms);

    /* Recalculate filter coefficients */
    aided_tl_retune(&data->tl_state, 1e3 / data->int_ms,
                    l->code_bw, l->code_zeta, l->code_k,
                    l->carr_to_code,
                    l->carr_bw, l->carr_zeta, l->carr_k,
                    l->carr_fll_aid_gain);

    lock_detect_reinit(&data->lock_detect,
                       lock_detect_params.k1 * data->int_ms,
                       lock_detect_params.k2,
                       /* TODO: Should also adjust lp and lo? */
                       lock_detect_params.lp, lock_detect_params.lo);

    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;

    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_early,
                             common_data->carrier_freq,
                             common_data->cn0);
  }

  u32 chips_to_correlate = (1 == data->int_ms) ?
                           L1CA_TRACK_MS_TO_CHIPS(1) :
                           L1CA_TRACK_MS_TO_CHIPS(data->int_ms - 1);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate, chips_to_correlate);
}

/** Parse a string describing the tracking loop filter parameters into
    the loop_params_stage structs. */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for either one or two
      stages.  If the second is omitted, we'll use the same parameters
      as the first stage.*/

  struct loop_params loop_params_parse[2];

  const char *str = val;
  for (int stage = 0; stage < 2; stage++) {
    struct loop_params *l = &loop_params_parse[stage];

    int n_chars_read = 0;
    unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

    if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) , %n",
               &tmp,
               &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
               &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain,
               &n_chars_read) < 9) {
      log_error("Ill-formatted tracking loop param string.");
      return false;
    }
    l->coherent_ms = tmp;
    /* If string omits second-stage parameters, then after the first
       stage has been parsed, n_chars_read == 0 because of missing
       comma and we'll parse the string again into loop_params_parse[1]. */
    str += n_chars_read;

    if ((l->coherent_ms == 0)
        || ((20 % l->coherent_ms) != 0) /* i.e. not 1, 2, 4, 5, 10 or 20 */
        || (stage == 0 && l->coherent_ms != 1)) {
      log_error("Invalid coherent integration length.");
      return false;
    }
  }
  /* Successfully parsed both stages.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(loop_params_stage, loop_params_parse, sizeof(loop_params_stage));
  return true;
}

/** Parse a string describing the tracking loop phase lock detector
    parameters into the lock_detect_params structs. */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string.");
      return false;
  }
  /* Successfully parsed.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));
  return true;
}
