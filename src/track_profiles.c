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

#include <string.h>

/*
 * Configuration section: select which features are enabled here.
 */
/* #define TP_USE_1MS_PROFILES */
/* #define TP_USE_2MS_PROFILES */
#define TP_USE_5MS_PROFILES
/* #define TP_USE_10MS_PROFILES */
/* #define TP_USE_20MS_PROFILES */

/** Maximum number of supported satellite vehicles */
#define TP_MAX_SUPPORTED_SVS NUM_GPS_L1CA_TRACKERS
/** Helper macro for array size computation */
#define ARR_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/** Default C/N0 threshold in dB/Hz for keeping track */
#define TP_DEFAULT_CN0_USE_THRESHOLD  (31.f)
/** Default C/N0 threshold in dB/Hz for dropping track */
#define TP_DEFAULT_CN0_DROP_THRESHOLD (31.f)

/**
 * Per-satellite entry.
 *
 * The system keeps some tracking information for all satellites. Generally
 * the entry is per satellite vehicle, not per signal.
 *
 * TODO Make entry to support multiple bands.
 */
typedef struct {
  bool          used;              /**< Flag if the profile entry is in use */
  gnss_signal_t sid;               /**< Signal identifier. */
  tp_report_t   last_report;       /**< Last data from tracker */
  u32           profile_update:1;  /**< Flag if the profile update is required */
  u32           cur_profile_i:3;   /**< Index of the currently active profile (integration) */
  u32           next_profile_i:3;  /**< Index of the next selected profile (integration) */
  float         prev_val[4];       /**< Filtered counters: v,a,j,C/N0 */
  float         filt_val[4];       /**< Filtered counters: v,a,j,C/N0 */
  float         mean_acc[4];       /**< Mean accumulators: v,a,j,C/N0 */
  u32           mean_cnt;          /**< Mean value divider */
  lp1_filter_t  lp_filters[4];     /**< Moving average filters: v,a,j,C/N0 */
  u32           time;              /**< Tracking time */
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
 * Lock detector profile
 */
static const tp_lock_detect_params_t ld_params_disable = {
  .k1 = 0.02f,
  .k2 = 1e-6f,
  .lp = 1,
  .lo = 1
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
/** Tracking profile for normal/1ms/pipelining */
static const tp_loop_params_t loop_params_1ms = {
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
static const tp_loop_params_t loop_params_5ms = {
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */

  .coherent_ms = 5,
  .carr_bw = 50,
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
  .carr_bw = 25,
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
static const tp_loop_params_t loop_params_20ms = {
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */

  .coherent_ms = 20,
  .carr_bw = 5, // 10/.9 is good; 5(1. is better
  .carr_zeta = .9f,
  .carr_k = 2.f,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .9f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_20MS_PROFILES */

/**
 * Vector of possible loop parameters.
 *
 * Entries do not have to have particular order, but the entry index shall
 * match the TP_LP_IDX_XYZ enumeration value.
 */
static const tp_loop_params_t *loop_params[] = {
  &loop_params_initial,
#ifdef TP_USE_1MS_PROFILES
  &loop_params_1ms,
#endif /* TP_USE_1MS_PROFILES */
#ifdef TP_USE_2MS_PROFILES
  &loop_params_2ms,
#endif /* TP_USE_2MS_PROFILES */
#ifdef TP_USE_5MS_PROFILES
  &loop_params_5ms,
#endif /* TP_USE_5MS_PROFILES */
#ifdef TP_USE_10MS_PROFILES
  &loop_params_10ms,
#endif /* TP_USE_10MS_PROFILES */
#ifdef TP_USE_20MS_PROFILES
  &loop_params_20ms
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
  TP_LP_IDX_INI,   /**< Initial state: very high noise bandwidth, high
                    * dynamics. */
#ifdef TP_USE_1MS_PROFILES
  TP_LP_IDX_1MS,   /**< 1MS pipelining integration. */
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
  TP_LP_IDX_2MS,   /**< 2MS pipelining integration. */
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  TP_LP_IDX_5MS,   /**< 5MS 1+N integration. */
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  TP_LP_IDX_10MS,  /**< 10MS 1+N integration. */
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  TP_LP_IDX_20MS   /**< 20MS 1+N integration. */
#endif /* TP_USE_20MS_PROFILES */
};

/**
 * State transition matrix.
 *
 * Matrix is two-dimensional: first dimension enumerates integration times,
 * second dimension is the dynamics profile.
 */
static const u8 profile_matrix[][1] = {
  {TP_LP_IDX_INI},

#ifdef TP_USE_1MS_PROFILES
  {TP_LP_IDX_1MS},
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
  {TP_LP_IDX_2MS},
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  {TP_LP_IDX_5MS},
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  {TP_LP_IDX_10MS},
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  {TP_LP_IDX_20MS}
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
    speed_mps = -(double)GPS_L1_LAMBDA_NO_VAC * doppler_hz;
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
  u8 idx = profile_matrix[profile->cur_profile_i][0];
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
  u8 profile_idx = profile_matrix[profile->cur_profile_i][0];
  log_debug_sid(profile->sid, "Activating profile %u [%u][%u])",
                profile_idx, profile->cur_profile_i, 0);

  config->lock_detect_params = ld_params_disable;
  config->loop_params = *loop_params[profile_idx];
  config->use_alias_detection = false;
  config->cn0_params = cn0_params_default;
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
  float speed, accel, jitter, cn0;

  /* Compute products */
  speed = compute_speed(profile->sid, data);
  accel = (profile->prev_val[0] - speed) * loop_freq;
  jitter = (profile->prev_val[1] - accel) * loop_freq;
  cn0 = data->cn0;

  /* Store new unfiltered values */
  profile->prev_val[0] = speed;
  profile->prev_val[1] = accel;
  profile->prev_val[2] = jitter;
  profile->prev_val[3] = cn0;

  /* Update RMS counters */
  profile->mean_acc[0] += speed * speed;
  profile->mean_acc[1] += accel * accel;
  profile->mean_acc[2] += jitter * jitter;
  profile->mean_acc[3] += cn0 * cn0;
  profile->mean_cnt += 1;

  /* Update moving average counters */
  speed = lp1_filter_update(&profile->lp_filters[0], speed);
  accel = (profile->filt_val[0] - speed) * loop_freq;
  jitter = (profile->filt_val[1] - accel) * loop_freq;
  /* Currently we don't additionally filter acceleration and jitter:
   *
   * accel = lp1_filter_update(&profile->lp_filters[1], accel);
   * jitter = lp1_filter_update(&profile->lp_filters[2], jitter);
   */
  cn0 = lp1_filter_update(&profile->lp_filters[3], data->cn0);

  profile->filt_val[0] = speed;
  profile->filt_val[1] = accel;
  profile->filt_val[2] = jitter;
  profile->filt_val[3] = cn0;
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
  if (profile->time - profile->last_print_time >= 20000) {
    profile->last_print_time = profile->time;

    float div = 1.f;
    if (profile->mean_cnt > 0)
      div = 1.f / profile->mean_cnt;

    float s = sqrtf(profile->mean_acc[0] * div);
    float a = sqrtf(profile->mean_acc[1] * div);
    float j = sqrtf(profile->mean_acc[2] * div);
    float c = sqrtf(profile->mean_acc[3] * div);

    u8 lp_idx = profile_matrix[profile->cur_profile_i][0];
    log_info_sid(profile->sid,
                 "MV: T=%dms N=%d CN0=%.2f s=%.3f a=%.3f j=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 profile->mean_cnt,
                 c, s, a, j
                );
    log_info_sid(profile->sid,
                 "LF: T=%dms N=%d CN0=%.2f s=%.3f a=%.3f j=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 profile->mean_cnt,
                 profile->filt_val[3],
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
  /** TODO add C/N0 analyzes */
  /** TODO add stress detector */
  /** TODO add TCXO drift support */

  bool        must_change_profile = false;
  u8          next_profile_i      = 0;
  const char *reason              = "cn0 OK";
  const char *reason2             = "dynamics OK";

  /* First, check if the profile change is required:
   * - There must be no scheduled profile change.
   * - For an upgrade from initial state, a bit sync shall be achieved.
   */
  if (!profile->profile_update) {
    if (profile->cur_profile_i == 0 &&
        profile->last_report.bsync &&
        profile->last_report.olock) {
      /* Transition from 1ms integration into 2 to 20 ms integration */
      must_change_profile = true;
      next_profile_i = 1;
      reason = "bit sync";
    }
  }

  /* Perform profile change */
  if (must_change_profile) {
      /* Profile update scheduling:
       * - Mark profile as pending for change
       * - Specify profile configuration index
       * - Log the information
       */
      profile->profile_update = true;
      profile->next_profile_i = next_profile_i;

      u8 lp1_idx = profile_matrix[profile->cur_profile_i][0];
      u8 lp2_idx = profile_matrix[profile->next_profile_i][0];

      log_info_sid(profile->sid,
                   "Profile change: %dms [%d][%d]->%dms [%d][%d] r=%s/%s",
                   (int)loop_params[lp1_idx]->coherent_ms,
                   profile->cur_profile_i, 0,
                   (int)loop_params[lp2_idx]->coherent_ms,
                   profile->next_profile_i, 0,
                   reason, reason2
                   );
  }
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

      profile->last_report = *data;
      profile->filt_val[0] = compute_speed(sid, data);
      profile->filt_val[1] = 0.;
      profile->filt_val[2] = 0.;
      profile->filt_val[3] = data->cn0;

      profile->mean_acc[0] = profile->filt_val[0] * profile->filt_val[0];
      profile->mean_acc[1] = 0;
      profile->mean_acc[2] = 0;
      profile->mean_acc[3] = profile->filt_val[3] * profile->filt_val[3];
      profile->mean_cnt = 1;

      profile->cur_profile_i = 0;
      profile->next_profile_i = 0;

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
 *
 * \retval TP_RESULT_SUCCESS New tracking profile has been retrieved. The
 *                           tracking loop shall reconfigure it's components
 *                           and, possibly, change the operation mode.
 * \retval TP_RESULT_NO_DATA New tracking profile is not available. No further
 *                           actions are needed.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_profile(gnss_signal_t sid, tp_config_t *config)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != config && NULL != profile) {
    if (profile->profile_update) {
      profile->profile_update = 0;
      profile->cur_profile_i = profile->next_profile_i;
      get_profile_params(profile, config);
      init_profile_filters(profile);

      res = TP_RESULT_SUCCESS;
    } else {
      res = TP_RESULT_NO_DATA;
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
    profile->last_report = *data;
    profile->time += data->time_ms;

    update_stats(profile, data);
    print_stats(profile);
    check_for_profile_change(profile);

    res = TP_RESULT_SUCCESS;
  }
  return res;
}
