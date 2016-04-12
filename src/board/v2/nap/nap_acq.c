/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>

#include <ch.h>

#include <libswiftnav/logging.h>

#include "acq_channel.h"
#include "acq.h"
#include "sbp.h"
#include "sbp_utils.h"

/** \defgroup acq Acquisition
 * Do acquisition searches via interrupt driven scheduling of SwiftNAP
 * acquisition channel correlations and peak detection.
 * \{ */

static BSEMAPHORE_DECL(load_wait_sem, TRUE);

void acq_set_sid(gnss_signal_t sid)
{
  nap_acq_code_wr_blocking(sid);
  if (chBSemWaitTimeout(&load_wait_sem, 1000) == MSG_TIMEOUT) {
    log_error("acq: Timeout waiting for code load!");
  }
}

/** Schedule a load of samples into the acquisition channel's sample ram.
 * The load starts at the end of the next timing strobe and continues until the
 * ram is full, at which time an interrupt is raised to the STM. This interrupt
 * is cleared by clearing the load enable bit of the acquisition channel's
 * LOAD ENABLE register.
 *
 * \param count The value of the NAP's internal counter at which the timing
 *              strobe is to go low.
 */
bool acq_load(u32 count)
{
  /* Initialise semaphore in the taken state, the calling thread can then wait
   * for the load to complete by waiting on this semaphore. */

  nap_acq_load_wr_enable_blocking();
  nap_timing_strobe(count);
  if (chBSemWaitTimeout(&load_wait_sem, 1000) == MSG_TIMEOUT) {
    log_info("acq: Sample load timeout. Probably set timing strobe in the past.");
    return false;
  }
  return true;
}

/** Handle an acquisition load done interrupt from the NAP acquisition channel.
 * Clear the enable bit of the acquisition channel load register and change
 * the acquisition state to ACQ_LOADING_DONE.
 */
void acq_service_load_done()
{
  /* Release semaphore to signal to waiting thread that
   * the load is complete. */
  chBSemSignal(&load_wait_sem);
}


static SEMAPHORE_DECL(acq_pipeline_sem, NAP_ACQ_PIPELINE_STAGES);
static struct {
  struct {
    s16 cf;
  } pipeline[NAP_ACQ_PIPELINE_STAGES];
  u8 p_head;
  u8 p_tail;

  float power_acc;    /**< Sum of powers of all acquisition set points. */
  u64 best_power;     /**< Highest power of all acquisition set points. */
  s16 best_cf;        /**< Carrier freq corresponding to highest power. */
  u16 best_cp;        /**< Code phase corresponding to highest power. */
  u32 count;          /**< Total number of acquisition points searched. */
} acq_state;

/** Start a blocking acquisition search for a PRN over a code phase / carrier frequency range.
 * Translate the passed code phase and carrier frequency float values into
 * acquisition register values. Write values for the first acquisition to the
 * channel, and then write values for the next pipelined acquisition.
 * Note : Minimum cf_bin_width is determined by the acq. channel carrier phase
 *        register width, and is given by 1/NAP_ACQ_CARRIER_FREQ_UNTS_PER_HZ
 *
 * \param cp_min   Starting code phase of the first acquisition. (chips)
 * \param cp_max   Starting code phase of the last acquisition. (chips)
 * \param cf_min   Carrier frequency of the first acquisition. (Hz)
 * \param cf_max   Carrier frequency of the last acquisition. (Hz)
 * \param cf_bin_width Step size between each carrier frequency to search. (Hz)
 */
void acq_search_begin(float cf_min_, float cf_max_, float cf_bin_width)
{
  chSemObjectInit(&acq_pipeline_sem, NAP_ACQ_PIPELINE_STAGES);
  memset(&acq_state, 0, sizeof(acq_state));

  /* Calculate the range parameters in acq units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
  u16 cf_step = cf_bin_width * NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  if (cf_step < 1)
    cf_step = 1;
  s16 cf_min = cf_step*floor(cf_min_*NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ /
    (float)cf_step);
  s16 cf_max = cf_step*ceil(cf_max_*NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ /
    (float)cf_step);

  for (s16 cf = cf_min; cf <= cf_max; cf += cf_step) {
    if (chSemWaitTimeout(&acq_pipeline_sem, 1000) == MSG_TIMEOUT) {
      log_error("acq: Search timeout (cf = %d)!", cf);
    }
    acq_state.pipeline[acq_state.p_head].cf = cf;
    acq_state.p_head = (acq_state.p_head + 1) % NAP_ACQ_PIPELINE_STAGES;
    nap_acq_init_wr_params_blocking(cf);
  }

  for (int i = 0; i < NAP_ACQ_PIPELINE_STAGES; i++) {
    if (chSemWaitTimeout(&acq_pipeline_sem, 1000) == MSG_TIMEOUT) {
      log_error("acq: Search timeout!");
    }
  }
}

/** Handle an acquisition done interrupt from the NAP acquisition channel. */
void acq_service_irq(void)
{
  s16 cf = acq_state.pipeline[acq_state.p_tail].cf;
  acq_state.p_tail = (acq_state.p_tail + 1) % NAP_ACQ_PIPELINE_STAGES;

  u16 index_max;
  u16 corr_max;
  float ave;

  nap_acq_corr_rd_blocking(&index_max, &corr_max, &ave);
  acq_state.power_acc += ave;
  if (corr_max > acq_state.best_power) {
    acq_state.best_power = corr_max;
    acq_state.best_cf = cf;
    acq_state.best_cp = index_max;
  }
  acq_state.count++;

  chSemSignal(&acq_pipeline_sem);
}

/** Get the results of the acquisition search last performed.
 * Get the code phase, carrier frequency, and SNR of the acquisition with the
 * highest SNR of set of acquisitions last performed.
 *
 * \param cp  Code phase of the acquisition result
 * \param cf  Carrier frequency of the acquisition result
 * \param cn0 Estimated CN0 of the acquisition result
 */
void acq_get_results(float* cp, float* cf, float* cn0)
{
  *cp = 1023.0 - (float)(acq_state.best_cp % (1023 * NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP))
                  / NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP;
  *cf = (float)acq_state.best_cf / NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  /* "SNR" estimated by peak power over mean power. */
  float snr = (float)acq_state.best_power / (acq_state.power_acc / acq_state.count);

  /* If there's a failure in chain, which results in the FFT being fed zeros
   * output power could return zero. Potential failure modes:
   * 1. GPS Front End is misconfigured and returning zeros or has lifted pin
   * 2. PRN is incorrect or corrupted to zeros
   * or if the FPGA has a critical failure 
   * that causes it to not raise interrupts count could be zero.  Catch
   * this condition so we don't propagate NaN up through the stack */
  if ((acq_state.power_acc == 0) || (acq_state.count == 0)) {
    log_error("acq: Power or frequency bin count is 0, causing SNR to be NaN. "
              "(best=%" PRIu64 ", acc=%f, count=%" PRIu32 ")",
              acq_state.best_power, acq_state.power_acc, acq_state.count);
    *cn0 = 0;
  } else {
  *cn0 = 10 * log10(snr)
       + 10 * log10(1.0 / NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ); /* Bandwidth */
  }
}

/** \} */

