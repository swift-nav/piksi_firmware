/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdio.h>
#include <libsbp/sbp.h>

#include "board/nap/cw_channel.h"
#include "sbp.h"
#include "cw.h"

/** \defgroup cw CW Interference
 * Search for CW interference in raw IF sample data.
 * Searches for CW interference by scheduling CW channel correlations on
 * the SwiftNAP and (TODO) doing peak detection. These peaked can then be
 * filtered out using the IIR filter onboard the SwiftNAP.
 * \{ */

cw_state_t cw_state;

/** Callback to start a set of CW searches.
 * Allows host to directly control CW channel searches.
 */
void cw_start_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  cw_start_msg_t* start_msg = (cw_start_msg_t*)msg;
  cw_start(start_msg->freq_min, start_msg->freq_max, start_msg->freq_step);
}

/** Register CW callbacks. */
void cw_setup()
{
  static sbp_msg_callbacks_node_t cw_start_callback_node;
  sbp_register_cbk(SBP_MSG_CW_START, &cw_start_callback, &cw_start_callback_node);
}

/** Schedule a load of samples into the CW channel's sample ram.
 * The load starts at the end of the next timing strobe and continues until the
 * ram is full, at which time an interrupt is raised to the STM. This interrupt
 * is cleared by clearing the LOAD ENABLE bit of the CW channel's LOAD ENABLE
 * register.
 *
 * \param count The value of the NAP's internal counter at which the timing
 *              strobe is to go low.
 */
void cw_schedule_load(u32 count)
{
  cw_state.state = CW_LOADING;
  nap_cw_load_wr_enable_blocking();
  nap_timing_strobe(count);
}

/** Handle a CW load done interrupt from the NAP CW channel.
 * Clear the enable bit of the CW channel LOAD register and change the CW
 * state to CW_LOADING_DONE.
 */
void cw_service_load_done()
{
  nap_cw_load_wr_disable_blocking();
  cw_state.state = CW_LOADING_DONE;
}

/** Query the state of the CW channel sample ram loading.
 * \return 1 if loading has finished, 0 otherwise
 */
u8 cw_get_load_done()
{
  return (cw_state.state == CW_LOADING_DONE);
}

/** Query the state of the CW channel search.
 * \return 1 if the set of search correlations has finished, 0 otherwise.
 */
u8 cw_get_running_done()
{
  return (cw_state.state == CW_RUNNING_DONE);
}

/** Start a CW search over a given range.
 * Find the CW correlation power of a given set of CW frequencies.
 *
 * \param freq_min        Frequency of the first search point. (Hz)
 * \param freq_max        Frequency of the last seach point. (Hz)
 * \param freq_bin_width  Step size between each search point. (Hz)
 */
void cw_start(float freq_min, float freq_max, float freq_bin_width)
{
  /* Calculate the range parameters in cw units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
  cw_state.freq_step = ceil(freq_bin_width*NAP_CW_FREQ_UNITS_PER_HZ);
  cw_state.freq_min = freq_min*NAP_CW_FREQ_UNITS_PER_HZ;
  cw_state.freq_max = freq_max*NAP_CW_FREQ_UNITS_PER_HZ;

  /* Initialise our cw state struct. */
  cw_state.state = CW_RUNNING;
  cw_state.count = 0;
  cw_state.freq = cw_state.freq_min;

  /* Write first and second sets of detection parameters (for pipelining). */
  nap_cw_init_wr_params_blocking(cw_state.freq_min);
  /* TODO: If we are only searching one point then write disable here. */
  nap_cw_init_wr_params_blocking(cw_state.freq + cw_state.freq_step);
}

/** Handle a CW DONE interrupt from the CW channel.
 * Record the correlations from the last CW search and write the next CW
 * frequency to the CW_INIT register. If this is one of the last two interrupts
 * for this search set, set the DISABLE bit of to the CW INIT register.
 */
void cw_service_irq()
{
  u64 power;
  corr_t cs;

  switch(cw_state.state)
  {
    default:
      /*
       * If we get an interrupt when we are not running, disable the CW channel.
       * This will also clear the interrupt.
       */
      nap_cw_init_wr_disable_blocking();
      break;

    case CW_RUNNING:
      /* Read in correlations. */
      nap_cw_corr_rd_blocking(&cs);

      power = (u64)cs.I*(u64)cs.I + (u64)cs.Q*(u64)cs.Q;

      if (cw_state.count < SPECTRUM_LEN) {
        cw_state.spectrum_power[cw_state.count] = power;
        cw_send_result(cw_state.freq, power);
      }
      cw_state.count++;

      /*
       * Write the next pipelined CW frequency to NAP's CW channel. If
       * this is one of the final two interrupts to be serviced, write to set
       * the CW's channel INIT register disable bit.
       */
      cw_state.freq += cw_state.freq_step;
			if (cw_state.freq >= (cw_state.freq_max + cw_state.freq_step)) {
        /* 2nd disable write. Transition state. */
        nap_cw_init_wr_disable_blocking();
        cw_state.state = CW_RUNNING_DONE;
			} else if (cw_state.freq>= cw_state.freq_max) {
        /* 1st disable write */
        nap_cw_init_wr_disable_blocking();
			} else {
        /* Write next pipelined CW frequency */
        nap_cw_init_wr_params_blocking(cw_state.freq + cw_state.freq_step);
			}

      break;
  }
}

/** Send results of a CW search point back to the host via the SBP interface.
 *
 * \param freq  Frequency of the CW correlation
 * \param power Magnitude of the CW correlation
 */
void cw_send_result(float freq, u64 power)
{
  struct {
    float freq;
    u64 power;
  } msg;

  msg.freq = freq;
  msg.power = power;

  sbp_send_msg(SBP_MSG_CW_RESULTS, sizeof(msg), (u8*)&msg);
}

/** Get a point from the CW correlations array
 *
 * \param freq  Pointer to float where frequency correlation index will be put
 * \param power Pointer to u64 where magnitude of the correlation index will be put
 * \param index Correlation array index to get freq and power from
 */
void cw_get_spectrum_point(float* freq, u64* power, u16 index)
{
	*freq = 0;
	*power = cw_state.spectrum_power[index];
}

/** \} */
