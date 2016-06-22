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

#ifndef SWIFTNAV_NAP_CONSTANTS_H
#define SWIFTNAV_NAP_CONSTANTS_H

/**
 * @brief   The sampling rate of the samples coming off the frontend
 */
#define NAP_FRONTEND_SAMPLE_RATE_Hz                                (99.375e6)

/**
 * @brief   The sample rate decimation used by acquisition
 */
#define NAP_ACQ_DECIMATION_RATE                                          (12)
/**
 * @brief   The acquisition sample rate after decimation
 * @note    This is the effective sampling rate of the acquisition results
 */
#define NAP_ACQ_SAMPLE_RATE_Hz             (NAP_FRONTEND_SAMPLE_RATE_Hz  \
		                                            / NAP_ACQ_DECIMATION_RATE)

/**
 * @brief   The sample rate decimation used by tracking channels
 */
#define NAP_TRACK_DECIMATION_RATE                                         (4)
/**
 * @brief   The tracking channel sample rate after decimation
 * @note    This is the effective sampling rate of the tracking results
 */
#define TRACK_SAMPLE_FREQ                   (NAP_FRONTEND_SAMPLE_RATE_Hz \
                                                 / NAP_TRACK_DECIMATION_RATE)

/**
 * @brief   The phase increment used to mix the frontend samples to baseband
 * @note    4294967296 is 2^32 and the .5 is for rounding
 */
#define NAP_FE_L1CA_BASEBAND_MIXER_PINC   (u32)(14.58e6 * 4294967296.0 \
                                          / NAP_FRONTEND_SAMPLE_RATE_Hz + 0.5)

/**
 * @brief   The phase increment used to mix the frontend samples to baseband
 * @note    4294967296 is 2^32 and the .5 is for rounding
 */
#define NAP_FE_L2C_BASEBAND_MIXER_PINC    (u32)(7.4e6 * 4294967296.0 \
                                          / NAP_FRONTEND_SAMPLE_RATE_Hz + 0.5)

#define NAP_KEY_LENGTH                                                   (16)

#define NAP_VERSION_STRING_OFFSET                                         (4)
#define NAP_VERSION_STRING_LENGTH                                        (52)

#define NAP_DNA_OFFSET                                                   (56)
#define NAP_DNA_LENGTH                                                    (8)

#endif /* SWIFTNAV_NAP_CONSTANTS_H */
