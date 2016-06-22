/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "board.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "track.h"
#include "main.h"

#include "track.h"

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CM_INTERNAL
#include "track_gps_l2cm.h"

#include "track_api.h"
#include "decode.h"
#include "manage.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>
#include "math.h"

#include "settings.h"
#include "signal.h"

/** L2C coherent integration time [ms] */
#define L2C_COHERENT_INTEGRATION_TIME_MS 20

#define L2CM_ALIAS_LOCK_DETECT_1ST (1 << 0)
#define L2CM_ALIAS_LOCK_DETECT_2ND (1 << 1)
#define L2CM_ALIAS_LOCK_DETECT_BOTH \
  (L2CM_ALIAS_LOCK_DETECT_1ST | L2CM_ALIAS_LOCK_DETECT_2ND)
#define L2CM_RUN_TL                (1 << 2)

struct fsm_states {
  float t_diff_s; /* the time difference used by the alias lock detector [s]*/
  u32 startup_int_time; /* start-up integration time [chips] */
  u32 alias_acc_length; /* alias detector accumulation length */
  struct {
    u32 int_time;  /* integration time [chips] */
    u8 next_fsm_state; /* next index in fsm_states::states array */
    u8 flags;      /* bit-field of L2CM_... flags */
  } states[23];
};

static const struct fsm_states fsm_states = {
  1023 * 1 / GPS_CA_CHIPPING_RATE, /* alias time delta  */
  300,  /* first integration time [chips] */
  500,     /* alias accumulation length [ms] */
  {
    /* start-up case */
    {423, 1, 0},  /* 0 */
    {1023, 2, 0}, /* 1 */
    {1023, 4, L2CM_ALIAS_LOCK_DETECT_1ST}, /* 2 */

    /* normal case */
    {723, 2, L2CM_ALIAS_LOCK_DETECT_BOTH | L2CM_RUN_TL},  /* 3 */
    {1023, 3, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 4 */
    {1023, 4, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 5 */
    {1023, 5, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 6 */
    {1023, 6, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 7 */
    {1023, 7, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 8 */
    {1023, 8, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 9 */
    {1023, 9, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 10 */
    {1023, 10, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 11 */
    {1023, 11, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 12 */
    {1023, 12, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 13 */
    {1023, 13, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 14 */
    {1023, 14, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 15 */
    {1023, 15, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 16 */
    {1023, 16, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 17 */
    {1023, 17, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 18 */
    {1023, 18, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 19 */
    {1023, 19, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 20 */
    {1023, 20, L2CM_ALIAS_LOCK_DETECT_BOTH}, /* 21 */
    {300, 3, L2CM_ALIAS_LOCK_DETECT_BOTH},  /* 22 */
  }
};

#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/*                        code: nbw zeta  k  carr_to_code
                       carrier:                     nbw zeta  k  fll_aid */
#define LOOP_PARAMS_MED "(20 ms, (1, 0.7, 1, 1200), (13, 0.7, 1, 5))"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS          "0.0247, 1.5, 50, 240"
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
static float track_cn0_drop_thres = 31.0; /* dBHz */

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS;
static bool use_alias_detection = true;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
  u8 fsm_state;                /**< The index of fsm_states::states array. */
} gps_l2cm_tracker_data_t;

static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);

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

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "loop_params",
                 loop_params_string,
                 TYPE_STRING, parse_loop_params);

  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "lock_detect_params",
                 lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_use",
          track_cn0_use_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_drop",
          track_cn0_drop_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "alias_detect",
          use_alias_detection, TYPE_BOOL);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param code_phase L1CA code phase [chips]
 * \param carrier_freq The current Doppler frequency for the L1 C/A channel
 * \param cn0 CN0 estimate for the L1 C/A channel
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init)
{
  /* First, get L2C capability for the SV from NDB */
  u32 l2c_cpbl;
  // TODO: uncomment this as soon as NDB gets available
  // ndb_gps_l2cm_l2c_cap_read(&l2c_cpbl);

  // TODO: remove this as soon as NDB gets available
  // GPS PRNs with L2C capability:
  // 01 03 05 06 07 08 09 10 12 15 17 24 25 26 27 29 30 31 32
  // as per http://tinyurl.com/zj5q62h
  l2c_cpbl = (u32)1 << (1 - 1);
  l2c_cpbl |= (u32)1 << (3 - 1);
  l2c_cpbl |= (u32)1 << (5 - 1);
  l2c_cpbl |= (u32)1 << (6 - 1);
  l2c_cpbl |= (u32)1 << (7 - 1);
  l2c_cpbl |= (u32)1 << (8 - 1);
  l2c_cpbl |= (u32)1 << (9 - 1);
  l2c_cpbl |= (u32)1 << (10 - 1);
  l2c_cpbl |= (u32)1 << (12 - 1);
  l2c_cpbl |= (u32)1 << (15 - 1);
  l2c_cpbl |= (u32)1 << (17 - 1);
  l2c_cpbl |= (u32)1 << (24 - 1);
  l2c_cpbl |= (u32)1 << (25 - 1);
  l2c_cpbl |= (u32)1 << (26 - 1);
  l2c_cpbl |= (u32)1 << (27 - 1);
  l2c_cpbl |= (u32)1 << (29 - 1);
  l2c_cpbl |= (u32)1 << (30 - 1);
  l2c_cpbl |= (u32)1 << (31 - 1);
  l2c_cpbl |= (u32)1 << (32 - 1);

  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2C signal from the SV is already in track */
  }

  if (0 == (l2c_cpbl & ((u32)1 << (sat - 1)))) {
    log_info_sid(sid, "SV does not support L2C signal");
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2C_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    /* recalculate doppler freq for L2 from L1*/
    .carrier_freq       = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
    .code_phase         = code_phase,
    .chips_to_correlate = fsm_states.startup_int_time,
    /* get initial cn0 from parent L1 channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  if (tracking_startup_request(&startup_params)) {
    log_info_sid(sid, "L2 CM handover done");
  } else {
    log_warn_sid(sid, "Failed to start L2C tracking");
  }
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage;

  assert(20 == l->coherent_ms);
  data->int_ms = l->coherent_ms;

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  /* Initialise C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3 / data->int_ms, common_data->cn0,
               CN0_EST_LPF_CUTOFF, 1e3 / data->int_ms);

  /* Initialize lock detector */
  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  alias_detect_init(&data->alias_detect,
                    fsm_states.alias_acc_length, fsm_states.t_diff_s);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);

  data->fsm_state = 0;
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

/** Runs alias detection logic
 * \param alias_detect Alias detector's state
 * \param I the value of in-phase arm of the correlator
 * \param Q the value of quadrature arm of the correlator
 * \return The frequency error of PLL [Hz]
 */
static s8 detect_alias(alias_detect_t *alias_detect, s32 I, s32 Q)
{
  float err = alias_detect_second(alias_detect, I, Q);
  float abs_err = fabs(err);
  int err_sign = (err > 0) ? 1 : -1;
  s8 correction;

  /* The expected frequency errors are +-(25 + N * 50) Hz
     For more details, see:
     https://swiftnav.hackpad.com/Alias-PLL-lock-detector-in-L2C-4fWUJWUNnOE */
  if (abs_err > 25.) {
    correction = 25;
    correction += 50 * (int)((abs_err - 25) / 50);
    abs_err -= correction;
    if ((abs_err + 25) > 50) {
      correction += 50;
    }
  } else if (abs_err > 25 / 2.) {
    correction = 25;
  } else {
    correction = 0;
  }

  return correction * err_sign;
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;
  u8 flags;
  corr_t cs[3];
  u32 sample_count;        /* Total num samples channel has tracked for. */
  double code_phase_early; /* Early code phase. */
  double carrier_phase;    /* Carrier phase in NAP register units. */

  tracker_correlations_read(channel_info->context, cs,
                            &sample_count,
                            &code_phase_early,
                            &carrier_phase);

  for(int i = 0; i < 3; i++) {
    data->cs[i].I += cs[i].I;
    data->cs[i].Q += cs[i].Q;
  }

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 fsm_states.states[data->fsm_state].int_time);

  flags = fsm_states.states[data->fsm_state].flags;
  data->fsm_state = fsm_states.states[data->fsm_state].next_fsm_state;

  if (flags & L2CM_ALIAS_LOCK_DETECT_2ND) {
    s8 err = 0;

    if (use_alias_detection &&
        data->lock_detect.outp && data->lock_detect.outo) {
      err = detect_alias(&data->alias_detect, cs[1].I, cs[1].Q);
    }

    if (err) {
      if (data->lock_detect.outp) {
        log_warn_sid(channel_info->sid,
                     "False phase lock detected. Err: %dHz", -err);
      }

      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  if (flags & L2CM_ALIAS_LOCK_DETECT_1ST) {
    alias_detect_first(&data->alias_detect, cs[1].I, cs[1].Q);
  }

  if (0 == (flags & L2CM_RUN_TL)) {
    return;
  }

  common_data->update_count += data->int_ms;

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           data->int_ms);

  /* Call the bit sync update API to do data decoding */
  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  /* Update C/N0 estimate */
  common_data->cn0 = cn0_est(&data->cn0_est,
                            data->cs[1].I / data->int_ms,
                            data->cs[1].Q / data->int_ms);
  if (common_data->cn0 > track_cn0_drop_thres) {
    common_data->cn0_above_drop_thres_count = common_data->update_count;
  }

  if (common_data->cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, data->cs[1].I, data->cs[1].Q,
                     data->int_ms);
  if (data->lock_detect.outo) {
    common_data->ld_opti_locked_count = common_data->update_count;
  }
  if (!data->lock_detect.outp) {
    common_data->ld_pess_unlocked_count = common_data->update_count;
  }

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* Output I/Q correlations using SBP if enabled for this channel */
  tracker_correlations_send(channel_info->context, data->cs);

  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = data->cs[2-i].I;
    cs2[i].Q = data->cs[2-i].Q;
  }

  aided_tl_update(&data->tl_state, cs2);
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq +
                                 GPS_CA_CHIPPING_RATE;

  for(int i = 0; i < 3; i++) {
    data->cs[i].I = 0;
    data->cs[i].Q = 0;
  }
}

/** Parse a string describing the tracking loop filter parameters into
 *  the loop_params_stage struct.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for one stage */

  struct loop_params loop_params_parse;

  const char *str = val;
  struct loop_params *l = &loop_params_parse;

  unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

  if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) ",
             &tmp,
             &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
             &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain
             ) < 9) {
    log_error("Ill-formatted tracking loop param string: %20s", str);
    return false;
  }
  l->coherent_ms = tmp;

  if (l->coherent_ms != L2C_COHERENT_INTEGRATION_TIME_MS) {
    log_error("Invalid coherent integration length for L2CM: %" PRIu8,
              l->coherent_ms);
    return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&loop_params_stage, &loop_params_parse, sizeof(loop_params_stage));

  return true;
}

/** Parse a string describing the tracking loop phase lock detector
 *  parameters into the lock_detect_params structs.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string: %20s", val);
      return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));

  return true;
}