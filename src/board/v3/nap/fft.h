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

#ifndef SWIFTNAV_FFT_H
#define SWIFTNAV_FFT_H

#include <libswiftnav/common.h>

#define FFT_LEN_LOG2_MIN 15
#define FFT_LEN_LOG2_MAX 15

#define FFT_LEN_MIN (1 << FFT_LEN_LOG2_MIN)
#define FFT_LEN_MAX (1 << FFT_LEN_LOG2_MAX)

typedef struct __attribute__((packed)) {
  s16 re;
  s16 im;
} fft_cplx_t;

typedef enum {
  FFT_DIR_BACKWARD = 0,
  FFT_DIR_FORWARD = 1
} fft_dir_t;

typedef enum {
  FFT_SAMPLES_INPUT_RF1_CH0 = 0,
  FFT_SAMPLES_INPUT_RF1_CH1 = 1,
  FFT_SAMPLES_INPUT_RF2_CH0 = 2,
  FFT_SAMPLES_INPUT_RF2_CH1 = 3,
  FFT_SAMPLES_INPUT_RF3_CH0 = 4,
  FFT_SAMPLES_INPUT_RF3_CH1 = 5,
  FFT_SAMPLES_INPUT_RF4_CH0 = 6,
  FFT_SAMPLES_INPUT_RF5_CH1 = 7
} fft_samples_input_t;

bool fft(const fft_cplx_t *in, fft_cplx_t *out, u32 len_log2,
         fft_dir_t dir, u32 scale_schedule);

bool fft_samples(fft_samples_input_t samples_input, fft_cplx_t *out,
                 u32 len_log2, fft_dir_t dir, u32 scale_schedule,
                 u32 *sample_count);

#endif /* SWIFTNAV_FFT_H */
