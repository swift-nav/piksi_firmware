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

#include "settings.h"
#include "signal.h"
#include "board.h"

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
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  cn0_filter_t    cn0_filt;    /**< C/N0 Filter */
  u8 int_ms;                   /**< Integration length. */
  bool short_cycle;            /**< Set to true when a short 1ms integration is requested. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
  tp_config_t params;          /**< Current stage parameters */
  tp_config_t next_params;     /**< Next stage parameters */
  bool        has_next_params; /**< Flag if stage transition is in progress */
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
    bool init)
{
  const tp_loop_params_t *l = &data->params.loop_params;
  const tp_lock_detect_params_t *ld = &data->params.lock_detect_params;

  data->int_ms = MIN(l->coherent_ms,
                     tracker_bit_length_get(channel_info->context));
  float loop_freq = 1000 / data->int_ms;

  if (init) {
    log_debug_sid(channel_info->sid, "Initializing TL");

    aided_tl_init(&(data->tl_state), loop_freq,
                  common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                  l->code_bw, l->code_zeta, l->code_k,
                  l->carr_to_code,
                  common_data->carrier_freq,
                  l->carr_bw, l->carr_zeta, l->carr_k,
                  l->carr_fll_aid_gain);
  } else {
    log_debug_sid(channel_info->sid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    aided_tl_retune(&data->tl_state, loop_freq,
                    l->code_bw, l->code_zeta, l->code_k,
                    l->carr_to_code,
                    l->carr_bw, l->carr_zeta, l->carr_k,
                    l->carr_fll_aid_gain);
  }
//  log_info_sid(channel_info->sid,
//               "TL: LF=%f, CODE(BW=%f Z=%f K=%f C2C=%f) CARR(BW=%f Z=%f K=%f G=%f)",
//               loop_freq,
//               l->code_bw, l->code_zeta, l->code_k,
//               l->carr_to_code,
//               l->carr_bw, l->carr_zeta, l->carr_k,
//               l->carr_fll_aid_gain);

  /* Initialize C/N0 estimator and filter */
  cn0_est_init(&data->cn0_est,       /* C/N0 estimator object */
               CN0_EST_BW_HZ,        /* BW */
               common_data->cn0,     /* Initial C/N0 */
               SAMPLE_FREQ,          /* Sampling frequency */
               loop_freq);           /* Loop filter frequency */
  cn0_filter_init(&data->cn0_filt,       /* Structure */
                  common_data->cn0,      /* Initial C/N0 */
                  CN0_EST_LPF_CUTOFF_HZ, /* Cut-off filter frequency */
                  loop_freq);            /* Loop filter frequency */

  lock_detect_init(&data->lock_detect,
                   ld->k1 * data->int_ms,
                   ld->k2,
                   ld->lp,
                   ld->lo);

  if (data->params.use_alias_detection) {
    u8 alias_detect_ms = data->int_ms;
    if (l->mode == TP_TM_ONE_PLUS_N)
      alias_detect_ms--;
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

    tp_tracking_start(channel_info->sid, &report, &data->params);
  }
  tracker_gps_l1ca_update_parameters(channel_info, common_data, data, true);
}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)common_data;
  (void)tracker_data;
  tp_tracking_stop(channel_info->sid);
}

static u8 compute_rollover_count(const gps_l1ca_tracker_data_t *data)
{
  u8 rollover_count;
  if (data->has_next_params) {
    /* Mode switch: first integration interval of the next configuration */
    switch (data->next_params.loop_params.mode) {
    case TP_TM_ONE_PLUS_N:
      rollover_count = 0;
      break;
    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    default:
      rollover_count = data->next_params.loop_params.coherent_ms - 1;
      break;
    }
  } else {
    /* Continuation of the current stage */
    switch (data->params.loop_params.mode) {
    case TP_TM_ONE_PLUS_N:
      rollover_count = data->short_cycle ?
                       0 :
                       data->params.loop_params.coherent_ms - 2;
      break;
    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    default:
      rollover_count = data->params.loop_params.coherent_ms - 1;
      break;
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
  switch (data->params.loop_params.mode)
  {
  case TP_TM_ONE_PLUS_N:
    next_ms = data->short_cycle ? data->params.loop_params.coherent_ms: 0;
    break;
  case TP_TM_PIPELINING:
  case TP_TM_IMMEDIATE:
  default:
    next_ms = data->params.loop_params.coherent_ms;
    break;
  }

  if (0 != next_ms &&
      tracker_next_bit_aligned(channel_info->context, next_ms)) {
    /* When the bit sync is available and the next integration interval is the
     * last one in the bit, check if the profile switch is required. */
    if (TP_RESULT_SUCCESS == tp_get_profile(channel_info->sid,
                                            &data->next_params)) {
      log_debug_sid(channel_info->sid,
                   "Changing tracking profile: current mode=%d ms=%d; "
                   "new mode=%d ms=%d",
                   data->params.loop_params.mode,
                   (int)data->params.loop_params.coherent_ms,
                   data->next_params.loop_params.mode,
                   (int)data->next_params.loop_params.coherent_ms);

      /* Initiate profile change */
      data->has_next_params = true;
    }
  } else {
    if (tp_has_new_profile(channel_info->sid)) {
        log_debug_sid(channel_info->sid,
                     "New profile is ready, but no bit sync");
    }
  }
}

static void mode_change_complete(const tracker_channel_info_t *channel_info,
                                 tracker_common_data_t *common_data,
                                 gps_l1ca_tracker_data_t *data)
{
  if (data->has_next_params) {
    /* If there is a stage transition in progress, update parameters for the
     * next iteration. */
    log_debug_sid(channel_info->sid,
                  "Reconfiguring tracking profile: new mode=%d, ms=%d",
                  data->next_params.loop_params.mode,
                  (int)data->next_params.loop_params.coherent_ms);

    data->params = data->next_params;
    data->has_next_params = false;
    data->short_cycle = false;
    tracker_gps_l1ca_update_parameters(channel_info, common_data, data, false);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;
  }
}


static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  u8 int_ms; /* Integration time for the currently reported value */
  bool sum_up = false;

  switch (data->params.loop_params.mode)
  {
  case TP_TM_ONE_PLUS_N:
    int_ms = data->short_cycle ? 1 : data->params.loop_params.coherent_ms - 1;
    sum_up = !data->short_cycle;
    break;
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
  default:
    int_ms = data->params.loop_params.coherent_ms;
  }

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  if (sum_up) {
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
    if (data->params.use_alias_detection) {
      alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);
    }
  }
  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);

  if (data->params.loop_params.mode == TP_TM_ONE_PLUS_N && data->short_cycle) {
    /* If we're doing long integrations, alternate between short and long
     * cycles.  This is because of FPGA pipelining and latency.  The
     * loop parameters can only be updated at the end of the second
     * integration interval and waiting a whole 20ms is too long.
     */

    /* We may change the integration time here, but only if the next long
     * integration period reaches bit boundary */
    mode_change_complete(channel_info, common_data, data);
    mode_change_init(channel_info, common_data, data);

    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate, compute_rollover_count(data));
    /* Next cycle is always long */
    data->short_cycle = false;
    return;
  }

  common_data->update_count += data->int_ms;

  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  float cn0_raw = cn0_est_update(&data->cn0_est, cs[1].I, cs[1].Q);
  common_data->cn0 = cn0_filter_update(&data->cn0_filt, cn0_raw);
  if (common_data->cn0 > data->params.cn0_params.track_cn0_drop_thres)
    common_data->cn0_above_drop_thres_count = common_data->update_count;

  if (common_data->cn0 < data->params.cn0_params.track_cn0_use_thres) {
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
    if (data->params.loop_params.coherent_ms > 1) {
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
  if (data->params.use_alias_detection &&
      (data->lock_detect.outp ||
       (data->lock_detect.outo))) {
    /* Last period integration time */
    u8 alias_ms = int_ms;

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
    report.time_ms = data->params.loop_params.coherent_ms;

    tp_report_data(channel_info->sid, &report);
  }

  mode_change_complete(channel_info, common_data, data);
  mode_change_init(channel_info, common_data, data);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate, compute_rollover_count(data));

  if (data->params.loop_params.mode == TP_TM_ONE_PLUS_N)
    data->short_cycle = !data->short_cycle;
}

