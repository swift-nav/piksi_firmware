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

#include "fft.h"

#include <assert.h>
#include <ch.h>

#include "axi_dma.h"
#include "nap_hw.h"
#include "nap_constants.h"

#define DATA_MEMORY_BARRIER() asm volatile ("dmb" : : : "memory")

#define FFT_TIMEOUT_ms (100)
#define TIMING_COMPARE_DELTA (NAP_FRONTEND_SAMPLE_RATE_Hz * 1e-3) /* 1ms */

static BSEMAPHORE_DECL(axi_dma_rx_bsem, 0);

static void axi_dma_tx_callback(bool success);
static void axi_dma_rx_callback(bool success);
static u32 length_points_get(u32 len_log2);
static void control_set_dma(void);
static void control_set_frontend_samples(fft_samples_input_t samples_input,
                                         u32 len_points);
static void control_set_raw_samples(u32 len_samples);
static void sample_stream_start(void);
static u32 sample_stream_snapshot_get(void);
static void config_set(fft_dir_t dir, u32 scale_schedule);
static void dma_start(const u8 *in, u8 *out, u32 len_bytes);
static bool dma_wait(void);

/** Callback for AXI DMA TX.
 */
static void axi_dma_tx_callback(bool success)
{
  (void)success;
  assert(success);
}

/** Callback for AXI DMA RX.
 */
static void axi_dma_rx_callback(bool success)
{
  /* Signal RX semaphore */
  chSysLockFromISR();
  if (success) {
    chBSemSignalI(&axi_dma_rx_bsem);
  } else {
    chBSemResetI(&axi_dma_rx_bsem, 0);
  }
  chSysUnlockFromISR();
}

/** Convert log2 length to length.
 *
 * \param len_log2        Log2 number of points.
 *
 * \return Number of points.
 */
static u32 length_points_get(u32 len_log2)
{
  return (1 << len_log2);
}

/** Set the ACQ control register for DMA input.
 */
static void control_set_dma(void)
{
  NAP->ACQ_CONTROL =
      (NAP_ACQ_CONTROL_DMA_INPUT_FFT      << NAP_ACQ_CONTROL_DMA_INPUT_Pos) |
      (NAP_ACQ_CONTROL_FFT_INPUT_DMA      << NAP_ACQ_CONTROL_FFT_INPUT_Pos) |
      (0                                  << NAP_ACQ_CONTROL_FRONTEND_Pos) |
      (0                                  << NAP_ACQ_CONTROL_LENGTH_Pos);
}

/** Set the ACQ control register for frontend samples input.
 *
 * \param samples_input   Frontend sample input to use.
 * \param len_points      Number of points.
 */
static void control_set_frontend_samples(fft_samples_input_t samples_input,
                                         u32 len_points)
{
  assert(!(len_points &
           ~(NAP_ACQ_CONTROL_LENGTH_Msk >> NAP_ACQ_CONTROL_LENGTH_Pos)));

  NAP->ACQ_CONTROL =
      (NAP_ACQ_CONTROL_DMA_INPUT_FFT      << NAP_ACQ_CONTROL_DMA_INPUT_Pos) |
      (NAP_ACQ_CONTROL_FFT_INPUT_FRONTEND << NAP_ACQ_CONTROL_FFT_INPUT_Pos) |
      ((samples_input)                    << NAP_ACQ_CONTROL_FRONTEND_Pos) |
      (len_points                         << NAP_ACQ_CONTROL_LENGTH_Pos);
}

/** Set the ACQ control register for raw samples input.
 *
 * \param len_samples     Number of samples.
 */
static void control_set_raw_samples(u32 len_samples)
{
  assert(!(len_samples &
           ~(NAP_ACQ_CONTROL_LENGTH_Msk >> NAP_ACQ_CONTROL_LENGTH_Pos)));

  NAP->ACQ_CONTROL =
      (NAP_ACQ_CONTROL_DMA_INPUT_SAMPLE_GRABBER
                                          << NAP_ACQ_CONTROL_DMA_INPUT_Pos) |
      (0                                  << NAP_ACQ_CONTROL_FFT_INPUT_Pos) |
      (0                                  << NAP_ACQ_CONTROL_RF_FE_Pos) |
      (0                                  << NAP_ACQ_CONTROL_RF_FE_CH_Pos) |
      (len_samples                        << NAP_ACQ_CONTROL_LENGTH_Pos);
}

/** Start streaming samples from the frontend or raw samples input.
 */
static void sample_stream_start(void)
{
  /* Set up timing compare */
  while (1) {
    chSysLock();
    u32 tc_req = NAP->TIMING_COUNT + TIMING_COMPARE_DELTA;
    NAP->ACQ_TIMING_COMPARE = tc_req;
    chSysUnlock();
    if (tc_req - NAP->ACQ_TIMING_SNAPSHOT <= TIMING_COMPARE_DELTA) {
      break;
    }
  }
}

/** Returns the sample count corresponding to the first sample of the most
 * recent sample stream.
 */
static u32 sample_stream_snapshot_get(void)
{
  return NAP->ACQ_START_SNAPSHOT;
}

/** Set the FFT config register.
 *
 * \param dir             FFT direction.
 * \param scale_schedule  Bitfield representing the scaling (right shift) to
 *                        be applied at each stage. Two bits per stage, Lsb
 *                        first.
 */
static void config_set(fft_dir_t dir, u32 scale_schedule)
{
  NAP->ACQ_FFT_CONFIG = (scale_schedule << NAP_ACQ_FFT_CONFIG_SCALE_Pos) |
                        (dir            << NAP_ACQ_FFT_CONFIG_DIR_Pos);
}

/** Start a DMA transfer.
 *
 * \param in              Input buffer, NULL if not used.
 * \param out             Output buffer.
 * \param len_bytes       Length of transfer (bytes).
 */
static void dma_start(const u8 *in, u8 *out, u32 len_bytes)
{
  if (in != 0) {
    /* Ensure that input accesses have completed */
    DATA_MEMORY_BARRIER();

    /* Start DMA transfer from in buffer to FPGA */
    axi_dma_write_begin(&AXIDMADriver1, in, len_bytes, axi_dma_tx_callback);
  }

  /* Make sure semaphore is in the TAKEN state. */
  chBSemWaitTimeout(&axi_dma_rx_bsem, TIME_IMMEDIATE);

  /* Start DMA transfer from FPGA to out buffer */
  axi_dma_read_begin(&AXIDMADriver1, out, len_bytes, axi_dma_rx_callback);
}

/** Wait for an FFT result DMA transfer to complete.
 *
 * \return True if a transfer was completed in time, false otherwise.
 */
static bool dma_wait(void)
{
  /* Wait for RX semaphore */
  if (chBSemWaitTimeout(&axi_dma_rx_bsem, MS2ST(FFT_TIMEOUT_ms)) != MSG_OK) {
    return false;
  }

  /* Ensure that output is not accessed before this point */
  DATA_MEMORY_BARRIER();

  return true;
}

/** Compute the FFT of a buffer of samples.
 *
 * \param in              Input buffer.
 * \param out             Output buffer.
 * \param len_log2        Log2 number of points.
 * \param dir             FFT direction.
 * \param scale_schedule  Bitfield representing the scaling (right shift) to
 *                        be applied at each stage. Two bits per stage, Lsb
 *                        first.
 *
 * \return True if the FFT was successfully computed, false otherwise.
 */
bool fft(const fft_cplx_t *in, fft_cplx_t *out, u32 len_log2,
         fft_dir_t dir, u32 scale_schedule)
{
  u32 len_bytes = length_points_get(len_log2) * sizeof(fft_cplx_t);
  config_set(dir, scale_schedule);
  control_set_dma();
  dma_start((const u8 *)in, (u8 *)out, len_bytes);
  return dma_wait();
}

/** Compute the FFT of a buffer of samples.
 *
 * \param samples_input   Frontend sample input to use.
 * \param out             Output buffer.
 * \param len_log2        Log2 number of points.
 * \param dir             FFT direction.
 * \param scale_schedule  Bitfield representing the scaling (right shift) to
 *                        be applied at each stage. Two bits per stage, Lsb
 *                        first.
 * \param sample_count    Output sample count of the first sample used.
 *
 * \return True if the FFT was successfully computed, false otherwise.
 */
bool fft_samples(fft_samples_input_t samples_input, fft_cplx_t *out,
                 u32 len_log2, fft_dir_t dir, u32 scale_schedule,
                 u32 *sample_count)
{
  u32 len_points = length_points_get(len_log2);
  u32 len_bytes = len_points * sizeof(fft_cplx_t);
  config_set(dir, scale_schedule);
  control_set_frontend_samples(samples_input, len_points);
  sample_stream_start();
  dma_start(0, (u8 *)out, len_bytes);
  bool result = dma_wait();
  *sample_count = sample_stream_snapshot_get();
  return result;
}

/** Retrieve a buffer of raw samples.
 *
 * \param out             Output buffer.
 * \param len_samples     Number of samples.
 * \param sample_count    Output sample count of the first sample.
 *
 * \return True if the samples were successfully retrieved, false otherwise.
 */
bool raw_samples_get(u8 *out, u32 len_samples, u32 *sample_count)
{
  control_set_raw_samples(len_samples);
  dma_start(0, out, len_samples);
  sample_stream_start();
  bool result = dma_wait();
  *sample_count = sample_stream_snapshot_get();
  return result;
}
