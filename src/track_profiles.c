/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <platform_signal.h>
#include "track_profiles.h"
#include "chconf_board.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

#include <board.h>
#include <platform_signal.h>
#include <nap/nap_common.h>
#include <nap/nap_hw.h>

#include <string.h>

/*
 * Configuration section: select which features are enabled here.
 */
#define TP_USE_1MS_PROFILES
#define TP_USE_2MS_PROFILES
#define TP_USE_5MS_PROFILES
#define TP_USE_10MS_PROFILES
#define TP_USE_20MS_PROFILES

/** Maximum number of supported satellite vehicles */
#define TP_MAX_SUPPORTED_SVS NUM_GPS_L1CA_TRACKERS
/** Helper macro for array size computation */
#define ARR_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/** Default C/N0 threshold in dB/Hz for keeping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_USE_THRESHOLD  (31.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD (31.f)

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD (23.f)
/** Fixed SNR offset for converting 1ms C/N0 to SNR */
#define TP_SNR_OFFSET  (-160.f)
/** C/N0 threshold for increasing integration time */
#define TP_SNR_THRESHOLD_MIN (35.f + TP_SNR_OFFSET)
/** C/N0 threshold for decreasing integration time */
#define TP_SNR_THRESHOLD_MAX (40.f + TP_SNR_OFFSET)
/** C/N0 threshold state lock counter */
#define TP_SNR_STATE_COUNT_LOCK (31)

/** Dynamics threshold for acceleration (stable) */
#define TP_ACCEL_THRESHOLD_LOW  (2.f)
/** Dynamics threshold for acceleration (unstable) */
#define TP_ACCEL_THRESHOLD_HIGH (9.f)
/** Dynamics threshold state lock counter */
#define TP_ACCEL_THRESHOLD_LOCK (31)

/** Profile lock time duration in ms. */
#define TP_CHANGE_LOCK_COUNTDOWN_MS (1250)

/**
 * Per-satellite entry.
 *
 * The system keeps some tracking information for all satellites. Generally
 * the entry is per satellite vehicle, not per signal.
 *
 * TODO Make entry to support multiple bands.
 */
typedef struct {
  gnss_signal_t sid;               /**< Signal identifier. */
  // tp_report_t   last_report;       /**< Last data from tracker */
  u32           used: 1;           /**< Flag if the profile entry is in use */
  u32           olock: 1;          /**<  */
  u32           plock: 1;          /**<  */
  u32           bsync: 1;          /**<  */
  u32           profile_update:1;  /**< Flag if the profile update is required */
  u32           cur_profile_i:3;   /**< Index of the currently active profile (integration) */
  u32           cur_profile_d:3;   /**< Index of the currently active profile (dynamics) */
  u32           next_profile_i:3;  /**< Index of the next selected profile (integration) */
  u32           next_profile_d:3;  /**< Index of the next selected profile (dynamics)  */
  u32           low_cn0_count:5;   /**< State lock counter for C/N0 threshold */
  u32           high_cn0_count:5;  /**< State lock counter for C/N0 threshold */
  u32           accel_count:5;     /**< State lock counter for dynamics threshold */
  u32           accel_count_idx:2; /**< State lock value for dynamics threshold */
  u32           lock_time_ms:16;   /**< Profile lock count down timer */
  float         cn0_offset;        /**< C/N0 offset in dB to tune thresholds */
  float         prev_val[4];       /**< Filtered counters: v,a,l,C/N0 */
  float         filt_val[4];       /**< Filtered counters: v,a,l,C/N0 */
  float         mean_acc[4];       /**< Mean accumulators: v,a,l,C/N0 */
  u32           mean_cnt;          /**< Mean value divider */
  lp1_filter_t  lp_filters[4];     /**< Moving average filters: v,a,l,C/N0 */
  u32           time_ms;              /**< Tracking time */
  u32           last_print_time;   /**< Last debug print time */
} tp_profile_internal_t;

/**
 * GPS satellite profiles.
 */
static tp_profile_internal_t profiles_gps1[TP_MAX_SUPPORTED_SVS] _CCM;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
  .track_cn0_drop_thres = TP_DEFAULT_CN0_DROP_THRESHOLD,
  .track_cn0_use_thres = TP_DEFAULT_CN0_USE_THRESHOLD
};

/**
 * Lock detector parameters
 */
enum {
  TP_LD_PARAMS_DISABLE,
  TP_LD_PARAMS_EXTRAOPT,
  TP_LD_PARAMS_OPT,
  TP_LD_PARAMS_NORMAL,
  TP_LD_PARAMS_PESS
};

/**
 * Lock detector profiles
 */
static const tp_lock_detect_params_t ld_params[] = {
  { 0.02f, 1e-6f,   1,  1, }, /* TP_LD_PARAMS_DISABLE */
  { 0.02,   0.8f, 150, 50, }, /* LD_PARAMS_EXTRAOPT */
  { 0.02,   1.1f, 150, 50, }, /* LD_PARAMS_OPT */
  { 0.05,   1.4f, 150, 50, }, /* LD_PARAMS_NORMAL */
  { 0.10f,  1.4f, 200, 50, }  /* TP_LD_PARAMS_PESS */
};

/**
 * Initial tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_initial = {
  /* "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5))" */
  .coherent_ms = 1,
  .carr_bw = 40,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 5,
  .mode = TP_TM_PIPELINING
};

#ifdef TP_USE_1MS_PROFILES
/** Tracking profile for stable/1ms/pipelining */
static const tp_loop_params_t loop_params_1ms_s = {
  .coherent_ms = 1,
  .carr_bw = 12,
  .carr_zeta = 1.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 1.f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
/** Tracking profile for normal/1ms/pipelining */
static const tp_loop_params_t loop_params_1ms_n = {
  .coherent_ms = 1,
  .carr_bw = 20,
  .carr_zeta = 1.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 1.f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
/** Tracking profile for unstable/1ms/pipelining */
static const tp_loop_params_t loop_params_1ms_u = {
  /* The configration is taken from previous 1ms profile:
   * "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5))"
   */
  .coherent_ms = 1,
  .carr_bw = 40,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
/**
 * 2 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_2ms = {
  /* "(2 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */

  .coherent_ms = 2,
  .carr_bw = 14,
  .carr_zeta = 1.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 1.f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
#endif /* TP_USE_2MS_PROFILES */
#ifdef TP_USE_5MS_PROFILES
/**
 * 5 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_5ms_s = {
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */

  .coherent_ms = 5,
  .carr_bw = 15,
  .carr_zeta = 1.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
/**
 * 5 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_5ms_n = {
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */

  .coherent_ms = 5,
  .carr_bw = 25,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_5MS_PROFILES */
#ifdef TP_USE_10MS_PROFILES
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_10ms = {
  /*  "(10 ms, (1, 0.7, 1, 1540), (30, 0.7, 1, 0))" */

  .coherent_ms = 10,
  .carr_bw = 16,
  .carr_zeta = .7f,
  .carr_k = 1.,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_10MS_PROFILES */
#ifdef TP_USE_20MS_PROFILES
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_20ms_s = {
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */

  .coherent_ms = 20,
  .carr_bw = 8, // 10/.9 is good; 5(1. is better
  .carr_zeta = .7f,
  .carr_k = 1.f,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
static const tp_loop_params_t loop_params_20ms_n = {
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */

  .coherent_ms = 20,
  .carr_bw = 12, // 10/.9 is good; 5(1. is better
  .carr_zeta = .9f,
  .carr_k = 1.f,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .9f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_20MS_PROFILES */

/**
 * Enumeration for the vertical dimension of profile matrix.
 *
 * Each entry here shall correspond to appropriate line in #profile_matrix.
 */
enum
{
  TP_PROFILE_ROW_INI=0,
#ifdef TP_USE_1MS_PROFILES
  TP_PROFILE_ROW_1MS,
#endif
#ifdef TP_USE_2MS_PROFILES
  TP_PROFILE_ROW_2MS,
#endif
#ifdef TP_USE_5MS_PROFILES
  TP_PROFILE_ROW_5MS,
#endif
#ifdef TP_USE_10MS_PROFILES
  TP_PROFILE_ROW_10MS,
#endif
#ifdef TP_USE_20MS_PROFILES
  TP_PROFILE_ROW_20MS,
#endif
  TP_PROFILE_ROW_COUNT,
  TP_PROFILE_ROW_FIRST = 1
};

/**
 * Dynamic profiles supported by the configuration.
 *
 * In the matrix of tracking loop parameters dynamics is identified by column
 * number.
 */
enum
{
  TP_PROFILE_DYN_LOW = 0,  /**< Low dynamics. Most stable parameters. */
  TP_PROFILE_DYN_MED,      /**< Medium dynamics. Averaged parameters. */
  TP_PROFILE_DYN_HIGH,     /**< High dynamics. Most relaxed parameters. */
  TP_PROFILE_DYN_COUNT,    /**< Total dynamic profile count */
  TP_PROFILE_DYN_INI = TP_PROFILE_DYN_MED /**< Initial dynamic profile */
};

/**
 * Vector of possible loop parameters.
 *
 * Entries do not have to have particular order, but the entry index shall
 * match the TP_LP_IDX_XYZ enumeration value.
 */
static const tp_loop_params_t *loop_params[] = {
  &loop_params_initial,
#ifdef TP_USE_1MS_PROFILES
  &loop_params_1ms_s, &loop_params_1ms_n, &loop_params_1ms_u,
#endif /* TP_USE_1MS_PROFILES */
#ifdef TP_USE_2MS_PROFILES
  &loop_params_2ms,
#endif /* TP_USE_2MS_PROFILES */
#ifdef TP_USE_5MS_PROFILES
  &loop_params_5ms_s, &loop_params_5ms_n, &loop_params_5ms_n,
#endif /* TP_USE_5MS_PROFILES */
#ifdef TP_USE_10MS_PROFILES
  &loop_params_10ms,
#endif /* TP_USE_10MS_PROFILES */
#ifdef TP_USE_20MS_PROFILES
  &loop_params_20ms_s, &loop_params_20ms_n, &loop_params_20ms_n,
#endif /* TP_USE_20MS_PROFILES */
};

/**
 * Enumeration of available profiles.
 *
 * Each entry here shall correspond to appropriate line in #loop_params. The
 * entries are used to index profiles in #profile_matrix table.
 */
enum
{
  TP_LP_IDX_INI,     /**< Initial state: very high noise bandwidth, high
                      * dynamics. */
#ifdef TP_USE_1MS_PROFILES
  TP_LP_IDX_1MS_S, /**< 1MS pipelining integration; stable */
  TP_LP_IDX_1MS_N, /**< 1MS pipelining integration; normal */
  TP_LP_IDX_1MS_U, /**< 1MS pipelining integration; unstable */
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
  TP_LP_IDX_2MS,     /**< 2MS pipelining integration. */
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  TP_LP_IDX_5MS_S,   /**< 5MS 1+N integration; stable. */
  TP_LP_IDX_5MS_N,   /**< 5MS 1+N integration; normal. */
  TP_LP_IDX_5MS_U,   /**< 5MS 1+N integration; unstable. */
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  TP_LP_IDX_10MS,    /**< 10MS 1+N integration. */
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  TP_LP_IDX_20MS_S, /**< 20MS 1+N integration; stable. */
  TP_LP_IDX_20MS_N, /**< 20MS 1+N integration; normal. */
  TP_LP_IDX_20MS_U  /**< 20MS 1+N integration; unstable. */
#endif /* TP_USE_20MS_PROFILES */
};

/**
 * State transition matrix.
 *
 * Matrix is two-dimensional: first dimension enumerates integration times,
 * second dimension is the dynamics profile.
 */
static const u8 profile_matrix[][TP_PROFILE_DYN_COUNT] = {
  {TP_LP_IDX_INI,  TP_LP_IDX_INI,  TP_LP_IDX_INI},

#ifdef TP_USE_1MS_PROFILES
  {TP_LP_IDX_1MS_S,  TP_LP_IDX_1MS_N,  TP_LP_IDX_1MS_U},
#endif

#ifdef TP_USE_2MS_PROFILES
  {TP_LP_IDX_2MS, TP_LP_IDX_2MS, TP_LP_IDX_2MS},
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  {TP_LP_IDX_5MS_S, TP_LP_IDX_5MS_N, TP_LP_IDX_5MS_U},
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  {TP_LP_IDX_10MS, TP_LP_IDX_10MS, TP_LP_IDX_10MS},
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  {TP_LP_IDX_20MS_S, TP_LP_IDX_20MS_N, TP_LP_IDX_20MS_U}
#endif /* TP_USE_20MS_PROFILES */
};

/**
 * Helper method for computing GNSS satellite speed from doppler.
 *
 * The method converts doppler frequency shift relative vector speed towards
 * the line of sight.
 *
 * \params[in] sid  GNSS satellite signal identifier.
 * \params[in] data Satellite tracking loop report data.
 *
 * \returns Speed in meters per second, or 0. on error.
 */
static double compute_speed(gnss_signal_t sid, const tp_report_t *data)
{
  double speed_mps = 0.;
  double doppler_hz = data->carr_freq; /* Carrier frequency is actually a
                                        * doppler frequency shift */

  switch (sid.code) {
  case CODE_GPS_L1CA:
    speed_mps = -(double)GPS_L1_LAMBDA * doppler_hz;
    break;
  case CODE_GPS_L2CM:
  default:
    /* Do not support */
    break;
  }

  return speed_mps;
}

/**
 * Helper method to (re-)initialize filters in satellite profile.
 *
 * \param[in,out] profile Satellite profile
 */
static void init_profile_filters(tp_profile_internal_t *profile)
{
  u8 idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
  const tp_loop_params_t *lp = loop_params[idx];
  float loop_freq = 1000 / lp->coherent_ms;
  for (size_t i = 0; i< ARR_SIZE(profile->lp_filters); ++i) {
    lp1_filter_init(&profile->lp_filters[i],
                    profile->filt_val[i],
                    0.6f,
                    loop_freq);
  }
}

/**
 * Allocates a new tracking profile structure for a satellite.
 *
 * Profiles identify physical GNSS satellites, not their signals.
 *
 * \param[in] sid  GNSS satellite signal identifier.
 *
 * \return Allocated profile pointer.
 * \retval NULL on error.
 */
static tp_profile_internal_t *allocate_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;

  /* Find unused entry */
  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (!profiles_gps1[i].used) {
      res = &profiles_gps1[i];
      break;
    }
  }

  /* If unused entry is found, mark it allocated and make default
   * initialization */
  if (NULL != res) {
    res->used = true;
    res->sid = sid;
  }
  return res;
}

/**
 * Locates profile for a given GNSS signal identifier.
 *
 * \param[in] sig GNSS satellite signal identifier.
 *
 * \return Allocated profile pointer.
 * \retval NULL on error.
 */
static tp_profile_internal_t *find_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;

  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (profiles_gps1[i].used &&
        profiles_gps1[i].sid.code == sid.code &&
        profiles_gps1[i].sid.sat == sid.sat) {
      res = &profiles_gps1[i];
      break;
    }
  }
  return res;
}

/**
 * Marks the signal as not tracked.
 *
 * The method locates profile for a signal and marks it as not tracked. If
 * necessary the profile is released.
 *
 * \param[in] sig GNSS satellite signal identifier.
 *
 * \return None
 */
static void delete_profile(gnss_signal_t sid)
{
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    /* Currently we support only one signal in profile, so simply mark the
     * profile as released. */
    memset(profile, 0, sizeof(*profile));
  }
}

/**
 * Helper method to obtain tracking loop parameters.
 *
 * The method generates tracking loop parameters according to selected
 * configuration.
 *
 * \param[in]  profile GNSS satellite profile.
 * \param[out] config  Container for computed configuration.
 *
 * \return None
 */
static void get_profile_params(tp_profile_internal_t *profile,
                               tp_config_t           *config)
{
  u8 profile_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
  log_debug_sid(profile->sid, "Activating profile %u [%u][%u])",
                profile_idx, profile->cur_profile_i, profile->cur_profile_d);

  const tp_lock_detect_params_t *p_ld_params = NULL;
  if (profile->cur_profile_i !=  TP_PROFILE_ROW_INI) {
    p_ld_params = &ld_params[TP_LD_PARAMS_NORMAL];
  } else {
    p_ld_params = &ld_params[TP_LD_PARAMS_DISABLE];
  }

  config->lock_detect_params = *p_ld_params;
  config->loop_params = *loop_params[profile_idx];
  config->use_alias_detection = false;

  tp_get_cn0_params(profile->sid, &config->cn0_params);
}

/**
 * Helper method to incorporate tracking loop information into statistics.
 *
 * \param[in,out] profile Satellite profile.
 * \param[in]     data    Data from tracking loop.
 *
 * \return None
 */
static void update_stats(tp_profile_internal_t *profile,
                         const tp_report_t *data)
{
  float loop_freq = 1000 / data->time_ms;
  float speed, accel, cn0, lock;

  profile->time_ms += data->time_ms;
  /* Profile lock time count down */
  if (profile->lock_time_ms > data->time_ms) {
    profile->lock_time_ms -= data->time_ms;
  } else {
    profile->lock_time_ms = 0;
  }

  profile->olock = data->olock;
  profile->plock = data->plock;
  profile->bsync = data->bsync;

  /* Compute products */
  speed = compute_speed(profile->sid, data);
  accel = (profile->prev_val[0] - speed) * loop_freq;
  cn0 = data->cn0_raw;
  if (data->lock_q != 0.f) {
    lock = data->lock_i / data->lock_q;
    if (lock > 100.f) {
      lock = 100.f;
    }
  } else {
    lock = 100.f;
  }

  /* Store new unfiltered values */
  profile->prev_val[0] = speed;
  profile->prev_val[1] = accel;
  profile->prev_val[2] = lock;
  profile->prev_val[3] = cn0;

  /* Update RMS counters */
  profile->mean_acc[0] += speed * speed;
  profile->mean_acc[1] += accel * accel;
  profile->mean_acc[2] += lock * lock;
  profile->mean_acc[3] += cn0 * cn0;
  profile->mean_cnt += 1;

  /* Update moving average counters */
  speed = lp1_filter_update(&profile->lp_filters[0], speed);
  accel = (profile->filt_val[0] - speed) * loop_freq;
  /* Currently we don't additionally filter acceleration and jitter:
   *
   * accel = lp1_filter_update(&profile->lp_filters[1], accel);
   * jitter = lp1_filter_update(&profile->lp_filters[2], jitter);
   * cn0 = lp1_filter_update(&profile->lp_filters[3], data->cn0);
   */
  lock = lp1_filter_update(&profile->lp_filters[2], lock);

  profile->filt_val[0] = speed;
  profile->filt_val[1] = accel;
  profile->filt_val[2] = lock;
  profile->filt_val[3] = data->cn0;
}

/**
 * Helper method to dump tracking statistics into log.
 *
 * The method logs average and RMS values for analyzes.
 *
 * \params[in] profile GNSS satellite profile
 *
 * \return None
 */
static void print_stats(tp_profile_internal_t *profile)
{
  if (profile->time_ms - profile->last_print_time >= 20000) {
    profile->last_print_time = profile->time_ms;

    float div = 1.f;
    if (profile->mean_cnt > 0)
      div = 1.f / profile->mean_cnt;

    float s = sqrtf(profile->mean_acc[0] * div);
    float a = sqrtf(profile->mean_acc[1] * div);
    float j = sqrtf(profile->mean_acc[2] * div);
    float c = sqrtf(profile->mean_acc[3] * div);

    u8 lp_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];

    log_info_sid(profile->sid,
                 "MRS: %dms CN0=%.2f (%.2f) VA=%.3f/%.3f l=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 c, c + TP_SNR_OFFSET,
                 s, a, j
                );
    log_info_sid(profile->sid,
                 "AVG: %dms CN0=%.2f (%.2f) VA=%.3f/%.3f l=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 profile->filt_val[3],
                 profile->filt_val[3] + TP_SNR_OFFSET,
                 profile->filt_val[0],
                 profile->filt_val[1],
                 profile->filt_val[2]
                );

    profile->mean_acc[0] = profile->prev_val[0] * profile->prev_val[0];
    profile->mean_acc[1] = profile->prev_val[1] * profile->prev_val[1];
    profile->mean_acc[2] = profile->prev_val[2] * profile->prev_val[2];
    profile->mean_acc[3] = profile->prev_val[3] * profile->prev_val[3];
    profile->mean_cnt = 1;
  }
}

/**
 * Internal method for evaluating profile change conditions.
 *
 * This method analyzes collected statistics and selects appropriate tracking
 * parameter changes.
 *
 * TODO This method shall be used in own thread.
 */
static void check_for_profile_change(tp_profile_internal_t *profile)
{
  /** TODO refactor as needed */
  /** TODO add TCXO drift support */

  bool        must_change_profile = false;
  bool        must_keep_profile   = false;
  u8          next_profile_i      = 0;
  u8          next_profile_d      = 0;
  const char *reason              = "cn0 OK";
  const char *reason2             = "dynamics OK";
  float       snr;
  float       acc;

  snr = profile->filt_val[3] + profile->cn0_offset + TP_SNR_OFFSET;
  acc = profile->filt_val[1];

  /* When we have a lock, and lock ratio is good, do not change the mode */
  // must_keep_profile = profile->olock && profile->filt_val[2] > 4.f;

  /* First, check if the profile change is required:
   * - There must be no scheduled profile change.
   * - For an upgrade from initial state, a bit sync shall be achieved.
   */
  if (!profile->profile_update) {
    if (profile->cur_profile_i == 0 &&
        profile->bsync &&
        profile->olock) {
      /* Transition from 1ms integration into 2 to 20 ms integration */
      must_change_profile = true;
      next_profile_i = 1;
      reason = "bit sync";
      profile->high_cn0_count = 0;
      profile->low_cn0_count = 0;
      profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
    }
    if (!must_keep_profile &&
        !must_change_profile &&
        profile->cur_profile_i != TP_LP_IDX_INI &&
        profile->lock_time_ms == 0) {
      /* When running over 1ms integration, there are four transitions
       * possible:
       * - increase integration time
       * - reduce integration time
       * - tighten loop parameters
       * - loosen loop parameters
       */
      /* Current C/N0 integration adjustments assume the integration times
       * are around factor of 2. This means there is ~3 dB/Hz gain/loss when
       * increasing/decreasing integration times.
       */

      if (snr >= TP_SNR_THRESHOLD_MAX) {
        /* SNR is high - look for relaxing profile */
        if (profile->cur_profile_i > TP_PROFILE_ROW_FIRST) {
          profile->high_cn0_count++;
          profile->low_cn0_count = 0;

          if (profile->high_cn0_count == TP_SNR_STATE_COUNT_LOCK) {
            reason="High C/N0";
            profile->high_cn0_count = 0;
            profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
            must_change_profile = true;
            next_profile_i = profile->cur_profile_i - 1;
            next_profile_d = profile->cur_profile_d;
          }
        }
      } else if (snr < TP_SNR_THRESHOLD_MIN) {
        /* SNR is low - look for more restricting profile */
        if (profile->cur_profile_i < TP_PROFILE_ROW_COUNT - 1) {
          profile->high_cn0_count = 0;
          profile->low_cn0_count++;
          if (profile->low_cn0_count == TP_SNR_STATE_COUNT_LOCK) {
            reason="Low C/N0";
            profile->low_cn0_count = 0;
            profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
            must_change_profile = true;
            next_profile_i = profile->cur_profile_i + 1;
            next_profile_d = profile->cur_profile_d;
          }
        }
      }
    }
  }
  /* Compute dynamics state.
   * Dynamics is evaluated as a delayed locked-in state as a function of
   * averaged acceleration.
   * At the moment three dynamic states are supported: low, medium and high
   * dynamics. They are separated by an acceleration trigger.
   */
  if (!must_keep_profile && profile->lock_time_ms == 0) {
    const char *dyn_reason = "";
    u8 dyn_idx = TP_PROFILE_DYN_INI;
    if (acc < TP_ACCEL_THRESHOLD_LOW) {
      dyn_idx = TP_PROFILE_DYN_LOW;
      dyn_reason = "Lower dynamics";
    } else if (acc > TP_ACCEL_THRESHOLD_HIGH) {
      dyn_idx = TP_PROFILE_DYN_HIGH;
      dyn_reason = "High dynamics";
    } else {
      dyn_idx = TP_PROFILE_DYN_MED;
      dyn_reason = "Normal dynamics";
    }
    if (profile->accel_count_idx == dyn_idx) {
      /* When the computed state matches last state, check if the state is
       * already active or will be active in next stage */
      if (!must_change_profile && profile->cur_profile_d == dyn_idx) {
        /* State is already active */
      } else if (must_change_profile && profile->next_profile_d == dyn_idx) {
        /* Next state is already selected */
      } else {
        profile->accel_count++;
        if (profile->accel_count == TP_ACCEL_THRESHOLD_LOCK) {
          /* State lock achieved. Reset counters. */
          profile->accel_count = 0;
          reason2 = dyn_reason;
          if (must_change_profile) {
            /* Profile change is already pending, just update the state */
            next_profile_d = dyn_idx;
          } else {
            /* Profile change due to dynamics state change only */
            next_profile_i = profile->cur_profile_i;
            next_profile_d = dyn_idx;
            must_change_profile = true;
            profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
          }
        }
      }
    } else {
      /* Dynamics state change reset: the previous dynamics state lock
       * to be reset*/
      profile->accel_count = 0;
      profile->accel_count_idx = dyn_idx;
    }
  } else {
    /* Dynamics state change is not permitted due to the lock */
    profile->accel_count = 0;
    profile->accel_count_idx = profile->cur_profile_d;
  }

  if (must_change_profile) {
    /* Perform profile change */
    /* Profile update scheduling:
     * - Mark profile as pending for change
     * - Specify profile configuration index
     * - Log the information
     */
    profile->profile_update = true;
    profile->next_profile_i = next_profile_i;
    profile->next_profile_d = next_profile_d;

    u8 lp1_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
    u8 lp2_idx = profile_matrix[profile->next_profile_i][profile->next_profile_d];

    log_info_sid(profile->sid,
                 "Profile change: %dms [%d][%d]->%dms [%d][%d] r=%s (%.2f)/%s (%.2f) l=%.2f",
                 (int)loop_params[lp1_idx]->coherent_ms,
                 profile->cur_profile_i, profile->cur_profile_d,
                 (int)loop_params[lp2_idx]->coherent_ms,
                 profile->next_profile_i, profile->next_profile_d,
                 reason, snr,
                 reason2, acc,
                 profile->filt_val[2]
                 );
  }
}

/**
 * Helper method for computing C/N0 offset.
 *
 * The method computes C/N0 offset for tracking loop in accordance to
 * integration period and tracking parameters.
 *
 * \param[in] profile GNSS satellite profile
 *
 * \return Computed C/N0 offset in dB/Hz.
 */
static float compute_cn0_offset(const tp_profile_internal_t *profile)
{
  u8 profile_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
  const tp_loop_params_t *lp = loop_params[profile_idx];
  float cn0_offset = 0;

  if (lp->coherent_ms > 1) {

    /* Denormalize C/N0.
     *
     * When integration time is higher, the tracking loop can keep tracking at
     * a much lower C/N0 values.
     *
     * TODO convert C/N0 to SNR to avoid confusion.
     */
    switch (lp->mode) {
    case TP_TM_ONE_PLUS_N:
      /* Very unfortunate, but the integrator handles N-1 milliseconds */
      cn0_offset = 10.f * log10f(lp->coherent_ms - 1);
      break;
    case TP_TM_PIPELINING:
    case TP_TM_IMMEDIATE:
    default:
      cn0_offset = 10.f * log10f(lp->coherent_ms);
      break;
    }
    log_debug_sid(profile->sid,
                  "CN0 offset %f @ %d", cn0_offset,
                  lp->coherent_ms);
  }

  return cn0_offset;
}

/**
 * Initializes the subsystem.
 *
 * This method shall be invoked before any other methods from the sybsystem.
 *
 * \return 0  On success.
 * \return -1 On error.
 */
tp_result_e tp_init()
{
  /** TODO refactor as needed */
  memset(profiles_gps1, 0, sizeof(profiles_gps1));

  return TP_RESULT_SUCCESS;
}

/**
 * Registers GNSS satellite in facility.
 *
 * The method registers GNSS signal and returns initial tracking parameters.
 *
 * \param[in]  sid    GNSS signal identifier.
 * \param[in]  data   Initial parameters.
 * \param[out] config Container for initial tracking parameters.
 *
 * \retval TP_RESULT_SUCCESS The satellite has been registered and initial
 *                           profile is returned.
 * \retval TP_RESULT_ERROR   On error.
 *
 * \sa tp_tracking_stop()
 */
tp_result_e tp_tracking_start(gnss_signal_t sid,
                              const tp_report_t *data,
                              tp_config_t *config)
{
  tp_result_e res = TP_RESULT_ERROR;

  if (NULL != config) {
    tp_profile_internal_t *profile = allocate_profile(sid);
    if (NULL != profile) {
      log_debug_sid(sid, "New tracking profile");

      // profile->last_report = *data;
      profile->filt_val[0] = compute_speed(sid, data);
      profile->filt_val[1] = 0.;
      profile->filt_val[2] = 0.;
      profile->filt_val[3] = data->cn0;

      profile->mean_acc[0] = 0;
      profile->mean_acc[1] = 0;
      profile->mean_acc[2] = 0;
      profile->mean_acc[3] = 0;
      profile->mean_cnt = 0;

      profile->cur_profile_i = TP_PROFILE_ROW_INI;
      profile->next_profile_i = TP_PROFILE_ROW_INI;
      profile->cur_profile_d = TP_PROFILE_DYN_INI;
      profile->next_profile_d = TP_PROFILE_DYN_INI;

      init_profile_filters(profile);

      get_profile_params(profile, config);

      res = TP_RESULT_SUCCESS;
    } else {
      log_error_sid(sid, "Can't allocate tracking profile");
    }
  }
  return res;
}

/**
 * Marks GNSS satellite as untracked.
 *
 * The method shall be invoked when tracking loop is terminated.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 *
 * \retval TP_RESULT_SUCCESS On success.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_tracking_stop(gnss_signal_t sid)
{
  tp_result_e res = TP_RESULT_ERROR;
  log_debug_sid(sid, "Removing tracking profile");
  delete_profile(sid);
  res = TP_RESULT_SUCCESS;
  return res;
}

/**
 * Retrieves new tracking profile if available.
 *
 * \param[in]  sid    GNSS signal identifier. This identifier must be registered
 *                    with a call to #tp_tracking_start().
 * \param[out] config Container for new tracking parameters.
 * \param[in]  commit Commit the mode change happened.
 *
 * \retval TP_RESULT_SUCCESS New tracking profile has been retrieved. The
 *                           tracking loop shall reconfigure it's components
 *                           and, possibly, change the operation mode.
 * \retval TP_RESULT_NO_DATA New tracking profile is not available. No further
 *                           actions are needed.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_profile(gnss_signal_t sid, tp_config_t *config, bool commit)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != config && NULL != profile) {
    if (profile->profile_update) {
      /* Do transition of current profile */
      if (commit) {
        profile->profile_update = 0;
        profile->cur_profile_i = profile->next_profile_i;
        profile->cur_profile_d = profile->next_profile_d;
        profile->cn0_offset = compute_cn0_offset(profile);
        init_profile_filters(profile);
      }

      /* Return data */
      get_profile_params(profile, config);

      res = TP_RESULT_SUCCESS;
    } else {
      res = TP_RESULT_NO_DATA;
    }
  }
  return res;
}

/**
 * Method for obtaining current C/N0 thresholds.
 *
 * \param[in]  sid    GNSS signal identifier. This identifier must be registered
 *                    with a call to #tp_tracking_start().
 * \param[out] cn0_params Container for C/N0 limits.
 *
 * \retval TP_RESULT_SUCCESS C/N0 thresholds have been retrieved.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_cn0_params(gnss_signal_t sid, tp_cn0_params_t *cn0_params)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != cn0_params && NULL != profile) {
    *cn0_params = cn0_params_default;

    /* Correction: higher integration time lowers thresholds linearly. For
     * example, 20ms integration has threshold by 13 dB lower, than for 1ms
     * integration. */
    cn0_params->track_cn0_drop_thres -= profile->cn0_offset;
    cn0_params->track_cn0_use_thres -= profile->cn0_offset;

    /* Currently, we don't have an algorithm that can differentiate tracked
     * signal from noise at C/N0 in range [21..23). This corresponds to SNR
     * of -145 dB/Hz and lower */
    /* TODO add heuristics to estimate that signal is tracked with SNR below
     * -145. */
    if (cn0_params->track_cn0_drop_thres < TP_HARD_CN0_DROP_THRESHOLD) {
      cn0_params->track_cn0_drop_thres = TP_HARD_CN0_DROP_THRESHOLD;
    }
    if (cn0_params->track_cn0_use_thres < TP_HARD_CN0_DROP_THRESHOLD) {
      cn0_params->track_cn0_use_thres = TP_HARD_CN0_DROP_THRESHOLD;
    }
  }
  return res;
}


/**
 * Method to check if there is a pending profile change.
 *
 * \param[in] sid GNSS satellite id.
 *
 * \retval true  New profile is available.
 * \retval false No profile change is required.
 */
bool tp_has_new_profile(gnss_signal_t sid)
{
  bool res = false;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    res = profile->profile_update != 0;
  }
  return res;
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 * \param[in] data Tracking loop report. This data is taken for analysis and
 *                 can be asynchronously.
 *
 * \retval TP_RESULT_SUCCESS on success.
 * \retval TP_RESULT_ERROR   on error.
 */
tp_result_e tp_report_data(gnss_signal_t sid, const tp_report_t *data)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != data && NULL != profile) {
    /* For now, we support only GPS L1 tracking data, and handle all data
     * synchronously.
     *
     * TODO schedule a message to own thread.
     */
    // profile->last_report = *data;

    update_stats(profile, data);
    print_stats(profile);
    check_for_profile_change(profile);

    res = TP_RESULT_SUCCESS;
  }
  return res;
}
