/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>

#include <ch.h>

#include <libswiftnav/time.h>
#include <libswiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "settings.h"
#include "main.h"
#include "timing.h"
#include "pps.h"

/** \defgroup pps Pulse-per-second (PPS)
 * Generate a pulse-per-second in alignment with GPS time.
 * \{ */

/** Number of microseconds the PPS will remain high (default: 200000). */
u32 pps_width_microseconds = PPS_WIDTH_MICROSECONDS;

static WORKING_AREA_CCM(wa_pps_thread, 256);
static void pps_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("PPS");

  while (TRUE) {
    if (time_quality == TIME_FINE) {
      gps_time_t t = get_current_time();
      t.tow = floor(t.tow) + 1;
      u64 next = round(gps2rxtime(&t) * PPS_NAP_CLOCK_RATIO) +
                       (PPS_NAP_CYCLES_OFFSET);
      nap_pps(next);
    }
    chThdSleepMilliseconds(PPS_THREAD_INTERVAL_MS);
  }
}

/** Set PPS width.
 * Sets the pulse width to a provided duration in microseconds.
 *
 * \param microseconds Pulse width in microseconds (1-999999).
 * \return Returns true if value is within valid range, false otherwise.
 */
bool pps_width(u32 microseconds)
{
  if (microseconds < 1 || microseconds >= 1e6) {
    log_info("Invalid PPS width. Valid range: 1-999999\n");
    return FALSE;
  }

  nap_pps_width(ceil((double)microseconds / (RX_DT_NOMINAL * 1e6)) - 1);
  return TRUE;
}

/** Settings callback for PPS width.
 * Updates the PPS width on NAP whenever the setting is changed.
 *
 * \param s Pointer to settings config.
 * \param val Pointer to new value.
 * \return Returns true if the change was successful, false otherwise.
 */
bool pps_width_changed(struct setting *s, const char *val)
{
  (void)s;
  (void)val;

  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return pps_width(pps_width_microseconds);
  }
  return FALSE;
}

/** Set up PPS generation.
 * Sets the default value for the PPS width and starts a thread to generate
 * the pulses.
 */
void pps_setup(void)
{
  pps_width(pps_width_microseconds);

  SETTING_NOTIFY("pps", "width", pps_width_microseconds, TYPE_INT,
                 pps_width_changed);

  chThdCreateStatic(wa_pps_thread, sizeof(wa_pps_thread),
                    NORMALPRIO+15, pps_thread, NULL);
}

/** \} */
