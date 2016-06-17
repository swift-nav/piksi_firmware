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
#include "track_api.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>

#include "settings.h"
#include "signal.h"
#include "board.h"
#include "platform_signal.h"
#include "track_profiles.h"

/* C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like */
#define CN0_EST_LPF_CUTOFF_HZ (.1f)
/* Noise bandwidth: GPS L1 1.023 * 2. Make 16dB offset. */
#define CN0_EST_BW_HZ         (2.046e6f)

/* Select one of the filters for C/N0 analysis: low-pass or Butterworth */
/* #define CN0_FILTER_BUTTERWORTH */
#define CN0_FILTER_LOWPASS

/* Configure C/N0 estimator algorithm */
#define cn0_est_init      cn0_est_bl_init
#define cn0_est_update    cn0_est_bl_update
/* Configure C/N0 value filter algorithm */
#ifdef CN0_FILTER_BUTTERWORTH
#define cn0_filter_t      bw2_filter_t
#define cn0_filter_init   bw2_filter_init
#define cn0_filter_update bw2_filter_update
#elif defined(CN0_FILTER_LOWPASS)
#define cn0_filter_t      lp1_filter_t
#define cn0_filter_init   lp1_filter_init
#define cn0_filter_update lp1_filter_update
#else
#error CN0 filter is not configured
#endif

typedef struct {
  aided_tl_state_t tl_state;               /**< Tracking loop filter state. */
  corr_t           cs[3];                  /**< EPL correlation results in
                                            *   correlation period. */
  cn0_est_state_t  cn0_est;                /**< C/N0 Estimator. */
  cn0_filter_t     cn0_filt;               /**< C/N0 Filter */
  alias_detect_t   alias_detect;           /**< Alias lock detector. */
  lock_detect_t    lock_detect;            /**< Phase-lock detector state. */
  u8               int_ms;                 /**< Current integration length. */
  u8               cycle_cnt: 5;           /**<  */
  u8               use_alias_detection: 1; /**< Flag for alias detection control */
  u8               alias_detect_first: 1;
  u8               tracking_mode: 3;       /**< Tracking mode */
  u8               has_next_params: 1;     /**< Flag if stage transition is in
                                            *   progress */
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
  for (u32 i=0; i<NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_update_parameters(
    const tracker_channel_info_t *channel_info,
    tracker_common_data_t *common_data,
    gps_l1ca_tracker_data_t *data,
    const tp_config_t *next_params,
    bool init)
{
  const tp_loop_params_t *l = &next_params->loop_params;
  const tp_lock_detect_params_t *ld = &next_params->lock_detect_params;

  //data->int_ms = MIN(l->coherent_ms,
  //                   tracker_bit_length_get(channel_info->context));
  data->tracking_mode = next_params->loop_params.mode;
  bool use_alias_detection = data->use_alias_detection;
  data->use_alias_detection = next_params->use_alias_detection;
  data->int_ms = next_params->loop_params.coherent_ms;
  data->has_next_params = false;
  switch (l->mode) {
  case TP_TM_SPLIT:
    data->cycle_cnt = data->int_ms - 1;
    break;
  case TP_TM_ONE_PLUS_N1:
    data->cycle_cnt = 1;
    break;
  case TP_TM_ONE_PLUS_N2:
    data->cycle_cnt = data->int_ms / 20;
    break;
  case TP_TM_INITIAL:
  case TP_TM_PIPELINING:
  case TP_TM_IMMEDIATE:
    data->cycle_cnt = 0;
    break;
  default:
    assert(false);
  }

  float loop_freq = 1000.f / data->int_ms;
  float cn0_lf = loop_freq;
  float ld_int_ms = data->int_ms;
  if (data->tracking_mode == TP_TM_SPLIT) {
    /* 1ms coherent interval split is used. */
    cn0_lf = 1000;
    ld_int_ms = 1;
  } else if (data->tracking_mode == TP_TM_ONE_PLUS_N2) {
    /* 20+ms coherent interval split is used. */
    cn0_lf = 50;
    // ld_int_ms = 20;
  }

  if (init) {
    log_debug_sid(channel_info->sid, "Initializing TL");

    aided_tl_init(&(data->tl_state), loop_freq,
                  common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                  l->code_bw, l->code_zeta, l->code_k,
                  l->carr_to_code,
                  common_data->carrier_freq,
                  l->carr_bw, l->carr_zeta, l->carr_k,
                  l->carr_fll_aid_gain);

    lock_detect_init(&data->lock_detect,
                     ld->k1 * ld_int_ms,
                     ld->k2,
                     ld->lp,
                     ld->lo);

  } else {
    log_debug_sid(channel_info->sid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    aided_tl_retune(&data->tl_state, loop_freq,
                    l->code_bw, l->code_zeta, l->code_k,
                    l->carr_to_code,
                    l->carr_bw, l->carr_zeta, l->carr_k,
                    l->carr_fll_aid_gain);
    lock_detect_reinit(&data->lock_detect,
                       ld->k1 * ld_int_ms,
                       ld->k2,
                       ld->lp,
                       ld->lo);
  }

  {


    /* Initialize C/N0 estimator and filter */
    cn0_est_init(&data->cn0_est,       /* C/N0 estimator object */
                 CN0_EST_BW_HZ,        /* BW */
                 common_data->cn0,     /* Initial C/N0 */
                 SAMPLE_FREQ,          /* Sampling frequency */
                 cn0_lf);              /* Loop filter frequency */
    cn0_filter_init(&data->cn0_filt,       /* Structure */
                    common_data->cn0,      /* Initial C/N0 */
                    CN0_EST_LPF_CUTOFF_HZ, /* Cut-off filter frequency */
                    cn0_lf);               /* Loop filter frequency */
  }

  data->alias_detect_first = true;

  if (data->use_alias_detection) {
    u8 alias_detect_ms = data->int_ms;
    if (l->mode == TP_TM_ONE_PLUS_N1 ||
        l->mode == TP_TM_ONE_PLUS_N2 ||
        l->mode == TP_TM_SPLIT)
      alias_detect_ms--;

    if (use_alias_detection)
      alias_detect_reinit(&data->alias_detect, 500 / alias_detect_ms,
                          alias_detect_ms * 1e-3f);
    else
      alias_detect_init(&data->alias_detect, 500 / alias_detect_ms,
                        alias_detect_ms * 1e-3f);
  }

}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l1ca_tracker_data_t *data = tracker_data;
  tp_config_t init_profile;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  {
    /* Do tracking report to manager */
    tp_report_t report;
    report.bsync = false;
    report.carr_freq = common_data->carrier_freq;
    report.code_phase_rate = common_data->code_phase_rate;
    report.cn0 = report.cn0_raw = common_data->cn0;
    report.olock = false;
    report.plock = false;
    report.sample_count = common_data->sample_count;
    report.time_ms = 0;

    tp_tracking_start(channel_info->sid, &report, &init_profile);
  }
  tracker_gps_l1ca_update_parameters(channel_info,
                                     common_data,
                                     data,
                                     &init_profile,
                                     true);
}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)common_data;
  (void)tracker_data;
  tp_tracking_stop(channel_info->sid);
}

static u8 compute_rollover_count(const tracker_channel_info_t *channel_info,
                                 const gps_l1ca_tracker_data_t *data)
{
  u8 rollover_count;
  if (data->has_next_params) {
    tp_config_t next_params;
    tp_get_profile(channel_info->sid, &next_params, false);

    /* Mode switch: first integration interval of the next configuration */
    switch (next_params.loop_params.mode) {
    case TP_TM_SPLIT:
    case TP_TM_ONE_PLUS_N1:
    case TP_TM_ONE_PLUS_N2:
      rollover_count = 0;
      break;
    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    case TP_TM_INITIAL:
    default:
      rollover_count = next_params.loop_params.coherent_ms - 1;
      break;
    }
  } else {
    /* Continuation of the current stage */
    switch (data->tracking_mode) {
    case TP_TM_SPLIT:
      rollover_count = 0;
      break;
    case TP_TM_ONE_PLUS_N1:
      rollover_count = data->cycle_cnt == 0 ?
                       0 :
                       data->int_ms - 2;
      break;
    case TP_TM_ONE_PLUS_N2:
      {
        u8 n_bits = data->int_ms / 20;
        if (data->cycle_cnt == n_bits - 1) {
          /* First interval is 1ms */
          rollover_count = 0;
        } else if (data->cycle_cnt == n_bits) {
          /* Second interval is 19ms */
          rollover_count = 18;
        } else {
          /* Other intervals are 20ms */
          rollover_count = 19;
        }
      }
      break;
    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    case TP_TM_INITIAL:
      rollover_count = data->int_ms - 1;
      break;
    default:
      assert(false);
    }
  }
  return rollover_count;
}

static void mode_change_init(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             gps_l1ca_tracker_data_t *data)
{
  /* Unused parameters */
  (void)common_data;

  /* Compute time of the currently integrated period */
  u8 next_ms = 0;
  switch (data->tracking_mode)
  {
  case TP_TM_SPLIT:
    next_ms = data->cycle_cnt + 2 == data->int_ms ? data->int_ms : 0;
    break;
  case TP_TM_ONE_PLUS_N1:
    next_ms = data->cycle_cnt == 0 ? data->int_ms: 0;
    break;
  case TP_TM_ONE_PLUS_N2:
    if (data->cycle_cnt == 1) {
      next_ms = 0;
    } else {
      next_ms = 20;
    }
    break;
  case TP_TM_INITIAL:
  case TP_TM_PIPELINING:
  case TP_TM_IMMEDIATE:
  default:
    next_ms = data->int_ms;
    break;
  }

  if (0 != next_ms &&
      tracker_next_bit_aligned(channel_info->context, next_ms)) {

    /* When the bit sync is available and the next integration interval is the
     * last one in the bit, check if the profile switch is required. */
    if (tp_has_new_profile(channel_info->sid)) {
      /* Initiate profile change */
      data->has_next_params = true;
    }
  }
}

/**
 * Finish profile switching operation.
 *
 * Method fetches new profile parameters and reconfigures as necessary.
 *
 * \param[in]     hannel_info Tracker channel data
 * \param[in,out] common_data Common data
 * \param[in,out] data        L1 C/A data
 *
 * \return None
 */
static void mode_change_complete(const tracker_channel_info_t *channel_info,
                                 tracker_common_data_t *common_data,
                                 gps_l1ca_tracker_data_t *data)
{
  if (data->has_next_params) {
    tp_config_t next_params;

    tp_get_profile(channel_info->sid, &next_params, true);

    /* If there is a stage transition in progress, update parameters for the
     * next iteration. */
    log_info_sid(channel_info->sid,
                  "Reconfiguring tracking profile: new mode=%d, ms=%d",
                  next_params.loop_params.mode,
                  (int)next_params.loop_params.coherent_ms);

    tracker_gps_l1ca_update_parameters(channel_info,
                                       common_data,
                                       data,
                                       &next_params,
                                       false);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;
  }
}

/**
 * Controls TL operation steps.
 *
 * The method updates the TL step according to tracking mode.
 *
 * \param[in,out] data Tracker data
 *
 * \return None
 */
static void update_cycle_counter(gps_l1ca_tracker_data_t *data)
{
  u8 cycles_cnt_limit;
  switch (data->tracking_mode) {
  case TP_TM_SPLIT:
    /* Special integration mode: each coherent integration is split into a
     * number of 1ms integrations. */
    cycles_cnt_limit = data->int_ms;
    break;
  case TP_TM_ONE_PLUS_N1:
    /* One plus N integrations.
     * Each cycle has two integrations: short (1ms) and long */
    cycles_cnt_limit = 2;
    break;
  case TP_TM_ONE_PLUS_N2:
    /* One plus N long integrations.
     * Each cycle has two integrations in the first bit, and one extra
     * integration per additional bit. */
    cycles_cnt_limit = data->int_ms / 20 + 1;
    break;
  case TP_TM_INITIAL:
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
    /* One cycle only. */
    cycles_cnt_limit = 0;
    break;
  default:
    assert(false);
  }
  if (cycles_cnt_limit > 0) {
    if (++data->cycle_cnt == cycles_cnt_limit) {
      data->cycle_cnt = 0;
    }
  }
}

enum
{
  SUM_UP_NONE,
  SUM_UP_ONE,
  SUM_UP_ONE_FLIP,
  SUM_UP_FLIP
};

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  u8 int_ms; /* Integration time for the currently reported value */
  u8 update_count_ms; /* Update counter. */
  u8 use_controller = true;
  u8 sum_up = SUM_UP_NONE;
  float ld_int_ms = data->int_ms;

  /* Determine, if the tracking channel EPL data shall be added or not. */
  switch (data->tracking_mode)
  {
  case TP_TM_SPLIT:
    int_ms = 1;
    sum_up = data->cycle_cnt > 1 ? SUM_UP_ONE : SUM_UP_NONE;
    ld_int_ms = 1;
    update_count_ms = data->int_ms;
    break;
  case TP_TM_ONE_PLUS_N1:
    int_ms = data->cycle_cnt == 0 ? 1 : data->int_ms - 1;
    sum_up = data->cycle_cnt != 0 ? SUM_UP_ONE : SUM_UP_NONE;
    update_count_ms = data->int_ms;
    break;
  case TP_TM_ONE_PLUS_N2:
    use_controller = false;
    if (data->cycle_cnt == 0) {
      int_ms = 1;
      sum_up = SUM_UP_NONE;
    } else if (data->cycle_cnt == 1) {
      int_ms = 19;
      sum_up = SUM_UP_ONE_FLIP;
    } else {
      int_ms = 20;
      sum_up = SUM_UP_FLIP;
    }
    if (data->cycle_cnt == data->int_ms / 20)
      use_controller = true;
    update_count_ms = 20;
    break;
  case TP_TM_INITIAL:
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
  default:
    int_ms = data->int_ms;
    update_count_ms = data->int_ms;
  }
  /* Prompt correlations for C/N0 estimator */
  corr_t cs_now[3]; /**< Correlations from FPGA */
  corr_t cs_bit[3]; /**< Sub-bit sum */

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  tracker_correlations_read(channel_info->context, cs_now,
                            &common_data->sample_count,
                            &common_data->code_phase_early,
                            &common_data->carrier_phase);

  switch (sum_up) {
  case SUM_UP_NONE:
    for (int i = 0; i < 3; ++i)
      cs_bit[i] = data->cs[i] = cs_now[i];
    break;
  case SUM_UP_ONE:
    for (int i = 0; i < 3; ++i) {
      cs_bit[i].I = data->cs[i].I += cs_now[i].I;
      cs_bit[i].Q = data->cs[i].Q += cs_now[i].Q;
    }
    break;
  case SUM_UP_ONE_FLIP:
    for (int i = 0; i < 3; ++i) {
      cs_bit[i].I = data->cs[i].I += cs_now[i].I;
      cs_bit[i].Q = data->cs[i].Q += cs_now[i].Q;
    }
    /* When using multi-bit coherent integration, invert values if needed. */
    if (data->cs[1].I < 0)
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I = -data->cs[i].I;
        data->cs[i].Q = -data->cs[i].Q;
      }
    break;
  case SUM_UP_FLIP:
    /* When using multi-bit coherent integration, invert new values if needed. */
    if (cs_now[1].I > 0)
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I += cs_bit[i].I = cs_now[i].I;
        data->cs[i].Q += cs_bit[i].Q = cs_now[i].Q;
      }
    else
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I -= cs_bit[i].I = cs_now[i].I;
        data->cs[i].Q -= cs_bit[i].Q = cs_now[i].Q;
      }
    break;
  default:
    assert(false);
  }
  if (data->tracking_mode == TP_TM_ONE_PLUS_N2 ) {
//    log_info_sid(channel_info->sid, "Scan: %d: n=%d/%d b=%d/%d s=%d/%d",
//                 data->cycle_cnt,
//                 cs_now[1].I, cs_now[1].Q,
//                 cs_bit[1].I, cs_bit[1].Q,
//                 data->cs[1].I, data->cs[1].Q);
  }

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);

  if ((data->tracking_mode == TP_TM_ONE_PLUS_N1 && data->cycle_cnt == 0) ||
      (data->tracking_mode == TP_TM_ONE_PLUS_N2 && data->cycle_cnt == 0) ||
      (data->tracking_mode == TP_TM_SPLIT && data->cycle_cnt < data->int_ms - 1)) {
    /* If we're doing long integrations, alternate between short and long
     * cycles.  This is because of FPGA pipelining and latency.  The
     * loop parameters can only be updated at the end of the second
     * integration interval and waiting a whole 20ms is too long.
     */

    if (data->tracking_mode == TP_TM_SPLIT) {
      float cn0_raw = cn0_est_update(&data->cn0_est, cs_now[1].I, cs_now[1].Q);
      common_data->cn0 = cn0_filter_update(&data->cn0_filt, cn0_raw);

      lock_detect_update(&data->lock_detect, cs_now[1].I, cs_now[1].Q, 1);
    }

    /* We may change the integration time here, but only if the next long
     * integration period reaches bit boundary */
    mode_change_complete(channel_info, common_data, data);
    mode_change_init(channel_info, common_data, data);

    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   compute_rollover_count(channel_info, data));

    update_cycle_counter(data);
    return;
  }

  /* Continue TL: below this line we may have both partial bit or multi-bit
   * integrations.
   *
   */

  common_data->update_count += update_count_ms;

  tracker_bit_sync_update(channel_info->context, update_count_ms, cs_bit[1].I);

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (data->tracking_mode != TP_TM_INITIAL) {
    tracker_correlations_send(channel_info->context, cs_bit);
  }

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  const corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  float cn0_raw = cn0_est_update(&data->cn0_est, cs_now[1].I, cs_now[1].Q);
  common_data->cn0 = cn0_filter_update(&data->cn0_filt, cn0_raw);
  tp_cn0_params_t cn0_params;
  tp_get_cn0_params(channel_info->sid, &cn0_params);

  if (common_data->cn0 > cn0_params.track_cn0_drop_thres ||
      data->lock_detect.outp) {
    /* When C/N0 is above a drop threshold or there is a pessimistic lock,
     * tracking shall continue.
     */
    common_data->cn0_above_drop_thres_count = common_data->update_count;
  }

  if (common_data->cn0 < cn0_params.track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  if (use_controller) {
    /* Run the loop filters. */

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs_now[1].I, cs_now[1].Q, ld_int_ms);
  if (data->lock_detect.outo)
    common_data->ld_opti_locked_count = common_data->update_count;
  if (!data->lock_detect.outp)
    common_data->ld_pess_unlocked_count = common_data->update_count;

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

#if 0
  if (data->tracking_mode == TP_TM_ONE_PLUS_N &&
      (channel_info->sid.sat == 1 || channel_info->sid.sat == 12)) {
    /* Extrapolating I/Q measurements from a shorted integration time into a
     * longer one. For example, in 1+N mode, the coherent N millisecond
     * integration shall be approximated into N+1 interval measurements to
     * linearize PLL/FLL models.
     * Generally, I branch grows according to N_samples^2 and Q branch grows
     * according to N_samples.
     */
    float k_q = (float)data->int_ms / (data->int_ms - 1);
    float k_i = k_q * k_q;
    for (u32 i = 0; i < 3; i++) {
      cs2[i].I = cs_now[2-i].I * k_i;
      cs2[i].Q = cs_now[2-i].Q * k_q;
    }
  } else {
  }
#else
#endif

    /* TODO: Make this more elegant. */
    correlation_t cs2[3];
    for (u32 i = 0; i < 3; i++) {
      cs2[i].I = cs[2-i].I;
      cs2[i].Q = cs[2-i].Q;
    }

    if (data->has_next_params) {
      /* Transitional state: when the next interval has a different integration
       * period, the controller will give wrong correction. Due to that the
       * input parameters are scaled to stabilize tracker.
       */
      u8 next_ms = 1;
      tp_get_next_coherent_ms(channel_info->sid, &next_ms);
      float k1 = (float)data->int_ms / next_ms;
      float k2 = k1 * k1;
      for (u32 i = 0; i < 3; i++) {
        cs2[i].I *= k1;
        cs2[i].Q *= k2;
      }
      aided_tl_update(&data->tl_state, cs2);
    } else {
      aided_tl_update(&data->tl_state, cs2);
    }
    common_data->carrier_freq = data->tl_state.carr_freq;
    common_data->code_phase_rate = data->tl_state.code_freq + GPS_CA_CHIPPING_RATE;

    /* Attempt alias detection if we have pessimistic phase lock detect, OR
       (optimistic phase lock detect AND are in second-stage tracking) */
    if (data->use_alias_detection &&
        ((data->tracking_mode == TP_TM_INITIAL && data->lock_detect.outp) ||
         (data->tracking_mode != TP_TM_INITIAL && data->lock_detect.outo))) {
      /* Last period integration time */
      u8 alias_ms = update_count_ms;

      if (data->alias_detect_first) {
        alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);
        data->alias_detect_first = false;
      } else {
        s32 I = (cs[1].I - data->alias_detect.first_I) / alias_ms;
        s32 Q = (cs[1].Q - data->alias_detect.first_Q) / alias_ms;
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
    } else if (data->alias_detect_first)
      data->alias_detect_first = false;


    {
      /* Do tracking report to manager */
      tp_report_t report;
      report.bsync = tracker_has_bit_sync(channel_info->context);
      report.carr_freq = common_data->carrier_freq;
      report.code_phase_rate = common_data->code_phase_rate;
      report.cn0_raw = cn0_raw;
      report.cn0 = common_data->cn0;
      report.olock = data->lock_detect.outo;
      report.plock = data->lock_detect.outp;
      report.lock_i = data->lock_detect.lpfi.y;
      report.lock_q = data->lock_detect.lpfq.y;
      report.sample_count = common_data->sample_count;
      report.time_ms = data->int_ms;

      tp_report_data(channel_info->sid, &report);
    }
  }

  mode_change_complete(channel_info, common_data, data);
  mode_change_init(channel_info, common_data, data);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 compute_rollover_count(channel_info, data));

  update_cycle_counter(data);
}

