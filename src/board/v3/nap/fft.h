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
#include <libswiftnav/signal.h>

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
  FFT_SAMPLES_INPUT_RF1 = 0,
  FFT_SAMPLES_INPUT_RF2 = 1,
  FFT_SAMPLES_INPUT_RF3 = 2,
  FFT_SAMPLES_INPUT_RF4 = 3,
} fft_samples_input_t;

bool fft(const fft_cplx_t *in, fft_cplx_t *out, u32 len_log2,
         fft_dir_t dir, u32 scale_schedule, constellation_t gnss);

bool fft_samples(fft_samples_input_t samples_input, fft_cplx_t *out,
                 u32 len_log2, fft_dir_t dir, u32 scale_schedule,
                 u32 *sample_count, constellation_t gnss);

#endif /* SWIFTNAV_FFT_H */
