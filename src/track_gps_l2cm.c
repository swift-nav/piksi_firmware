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

#include "track_gps_l2cm.h"
#include "track.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>

#include "settings.h"

#define NUM_GPS_L2CM_TRACKERS   12 //TODO: to be updated

/*  code: nbw zeta k carr_to_code
 carrier:                    nbw  zeta k fll_aid */

#define GPS_L2CM_LOOP_PARAMS_MED \
  "(20 ms, (1, 0.7, 1, 1200), (13, 0.7, 1, 5))" 

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS_PESS     "0.10, 1.4, 200, 50"
#define LD_PARAMS_NORMAL   "0.05, 1.4, 150, 50"
#define LD_PARAMS_OPT      "0.02, 1.1, 150, 50"
#define LD_PARAMS_EXTRAOPT "0.02, 0.8, 150, 50"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

#define CN0_EST_LPF_CUTOFF 5

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  u8 coherent_ms;
} loop_params_stage;

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 31.0; /* dBHz */
static float track_cn0_drop_thres = 31.0;

static char loop_params_string[] = GPS_L2CM_LOOP_PARAMS_MED;
static char lock_detect_params_string[] = LD_PARAMS_DISABLE;
static bool use_alias_detection = true;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  u32 code_phase_rate_fp;      /**< Code phase rate in NAP register units. */
  u32 code_phase_rate_fp_prev; /**< Previous code phase rate in NAP register units. */
  s32 carrier_freq_fp;         /**< Carrier frequency in NAP register units. */
  s32 carrier_freq_fp_prev;    /**< Previous carrier frequency in NAP register units. */
  u32 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
} gps_l2cm_tracker_data_t;

static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  const tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     const tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    const tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params_l2cm(struct setting *s, const char *val);
static bool parse_lock_detect_params_l2cm(struct setting *s, const char *val);

static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

static tracker_interface_list_element_t
tracker_interface_list_element_gps_l2cm = {
  .interface = &tracker_interface_gps_l2cm,
  .next = 0
};

void track_gps_l2cm_register(void)
{
  SETTING_NOTIFY("track", "loop_params", loop_params_string,
                 TYPE_STRING, parse_loop_params_l2cm);
  SETTING_NOTIFY("track", "lock_detect_params", lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params_l2cm);
  SETTING("track", "cn0_use", track_cn0_use_thres, TYPE_FLOAT);
  SETTING("track", "cn0_drop", track_cn0_drop_thres, TYPE_FLOAT);
  SETTING("track", "alias_detect", use_alias_detection, TYPE_BOOL);


  for (u32 i=0; i<NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  const tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{

  (void)channel_info;
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));

  tracker_channel_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage;

  data->int_ms = l->coherent_ms;

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  /*DT: I think NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ for L2C should be 
   * NAP_TRACK_NOMINAL_CODE_PHASE_RATE / 10.23e6, does HW team provide 
   * file (nap/track_channel.h) with these constants */
  data->code_phase_rate_fp = common_data->code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->carrier_freq_fp = (s32)(common_data->carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  data->carrier_freq_fp_prev = data->carrier_freq_fp;

  /* Initialise C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3/data->int_ms, common_data->cn0, CN0_EST_LPF_CUTOFF, 1e3/data->int_ms);

  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     const tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    const tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{

  gps_l2cm_tracker_data_t *data = tracker_data;


  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), channel_info->sid);

  tracker_channel_correlations_read(channel_info->context, data->cs,
                                      &data->corr_sample_count);
  alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);


  tracker_common_data_t updated_common_data = *common_data;


  updated_common_data.sample_count += data->corr_sample_count;
  updated_common_data.code_phase_early = (u64)updated_common_data.code_phase_early +
                           (u64)data->corr_sample_count
                             * data->code_phase_rate_fp_prev;
  updated_common_data.carrier_phase += (s64)data->carrier_freq_fp_prev
                           * data->corr_sample_count;
  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->carrier_freq_fp_prev = data->carrier_freq_fp;

  u8 int_ms = data->int_ms;
  updated_common_data.TOW_ms = tracker_tow_update(channel_info->context,
                                                  updated_common_data.TOW_ms,
                                                  int_ms);
 
  updated_common_data.update_count += data->int_ms;

  tracker_bit_sync_update(channel_info->context, int_ms, data->cs[1].I);

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  updated_common_data.cn0 = cn0_est(&data->cn0_est, cs[1].I/data->int_ms, cs[1].Q/data->int_ms);
  if (updated_common_data.cn0 > track_cn0_drop_thres)
    updated_common_data.cn0_above_drop_thres_count = updated_common_data.update_count;

  if (updated_common_data.cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_channel_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    updated_common_data.cn0_below_use_thres_count = updated_common_data.update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs[1].I, cs[1].Q, data->int_ms);
  if (data->lock_detect.outo)
    updated_common_data.ld_opti_locked_count = updated_common_data.update_count;
  if (!data->lock_detect.outp)
    updated_common_data.ld_pess_unlocked_count = updated_common_data.update_count;

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp)
    tracker_channel_ambiguity_unknown(channel_info->context);

  /* Run the loop filters. */

  /* TODO: Make this more elegant. */
  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs[2-i].I;
    cs2[i].Q = cs[2-i].Q;
  }

  /* Output I/Q correlations using SBP if enabled for this channel */
  tracker_channel_correlations_send(channel_info->context, cs);

  aided_tl_update(&data->tl_state, cs2);
  updated_common_data.carrier_freq = data->tl_state.carr_freq;
  updated_common_data.code_phase_rate = data->tl_state.code_freq + GPS_CA_CHIPPING_RATE;

  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->code_phase_rate_fp = updated_common_data.code_phase_rate
    * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ; /* DT: shouldn't it be changed? */

  data->carrier_freq_fp = updated_common_data.carrier_freq
    * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (use_alias_detection &&
     (data->lock_detect.outp || data->lock_detect.outo)) {
    s32 I = (cs[1].I - data->alias_detect.first_I) / (data->int_ms - 1);
    s32 Q = (cs[1].Q - data->alias_detect.first_Q) / (data->int_ms - 1);
    float err = alias_detect_second(&data->alias_detect, I, Q);
    if (fabs(err) > (250 / data->int_ms)) {
      if (data->lock_detect.outp) {
        log_warn("False phase lock detected on %s: err=%f", buf, err);
      }

      tracker_channel_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      updated_common_data.mode_change_count = updated_common_data.update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  tracker_channel_retune(channel_info->context, data->carrier_freq_fp,
                         data->code_phase_rate_fp,
                         data->int_ms); //DT: to check this

  /* Update common data */
  tracker_common_data_update(channel_info->context, &updated_common_data);
}

/** Parse a string describing the tracking loop filter parameters into
    the loop_params_stage structs. */
static bool parse_loop_params_l2cm(struct setting *s, const char *val)
{
  /** The string contains loop parameters for either one or two
      stages.  If the second is omitted, we'll use the same parameters
      as the first stage.*/

  struct loop_params loop_params_parse;

  const char *str = val;
  struct loop_params *l = &loop_params_parse;

  unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

  if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) ",
             &tmp,
             &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
             &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain
             ) < 9) {
    log_error("Ill-formatted tracking loop param string.");
    return false;
  }
  l->coherent_ms = tmp;
  /* If string omits second-stage parameters, then after the first
     stage has been parsed, n_chars_read == 0 because of missing
     comma and we'll parse the string again into loop_params_parse[1]. */

  if (l->coherent_ms != 20) {
    log_error("Invalid coherent integration length for L2CM.");
    return false;
  }
  /* Successfully parsed both stages.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(&loop_params_stage, &loop_params_parse, sizeof(loop_params_stage));
  return true;
}

/** Parse a string describing the tracking loop phase lock detector
    parameters into the lock_detect_params structs. 
DT: at the moment this parser is same like for L1CA, 
perhaps it's worth to reuse that one */
static bool parse_lock_detect_params_l2cm(struct setting *s, const char *val)
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
