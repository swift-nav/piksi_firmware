/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>

#define memory_pool_t MemoryPool
#include <ch.h>
#undef memory_pool_t

#include <libsbp/system.h>
#include <libsbp/version.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/dgnss_management.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/coord_system.h>

#include "board/nap/nap_common.h"
#include "board/frontend.h"
#include "peripherals/leds.h"
#include "main.h"
#include "sbp.h"
#include "manage.h"
#include "simulator.h"
#include "system_monitor.h"
#include "position.h"

#define WATCHDOG_THREAD_PERIOD_MS 15000
extern const WDGConfig board_wdg_config;

/* Maximum distance between calculated and surveyed base station single point
 * position for error checking.
 */
#define BASE_STATION_DISTANCE_THRESHOLD 15000

/* Time between sending system monitor and heartbeat messages in milliseconds */
static uint32_t heartbeat_period_milliseconds = 1000;
/* Use watchdog timer or not */
static bool use_wdt = true;

/* Build up the event mask for the thread activity watchdog.  Equivalent to
   EVENT_MASK(0) | EVENT_MASK(1) | ... | EVENT_MASK(WD_NOTIFY_NUM_THREADS) */
static eventmask_t thread_activity_mask = ((1UL << WD_NOTIFY_NUM_THREADS) - 1);

static thread_t *watchdog_thread_handle;

/* Base station mode settings. */
/* TODO: Relocate to a different file? */
static bool broadcast_surveyed_position = false;
static double base_llh[3];

/* Global CPU time accumulator, used to measure thread CPU usage. */
u64 g_ctime = 0;


u32 check_stack_free(thread_t *tp)
{
  u32 *stack = (u32 *)tp->p_stklimit;
  u32 i;
  for (i=0; i<65536/sizeof(u32); i++) {
    if (stack[i] != 0x55555555)
      break;
  }
  return 4 * (i - 1);
}

void send_thread_states()
{
  thread_t *tp = chRegFirstThread();
  while (tp) {
    msg_thread_state_t tp_state;
    u16 cpu = 1000.0f * tp->p_ctime / (float)g_ctime;
    tp_state.cpu = cpu;
    tp_state.stack_free = check_stack_free(tp);
    strncpy(tp_state.name, chRegGetThreadNameX(tp), sizeof(tp_state.name));
    sbp_send_msg(SBP_MSG_THREAD_STATE, sizeof(tp_state), (u8 *)&tp_state);

    tp->p_ctime = 0;  /* Reset thread CPU cycle count */
    tp = chRegNextThread(tp);
  }
  g_ctime = 0;
}

static THD_WORKING_AREA(wa_track_status_thread, 256);
static void track_status_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("track status");
  while (TRUE) {
    if (simulation_enabled()) {
      led_on(LED_GREEN);
      chThdSleepMilliseconds(500);
    } else {
      chThdSleepMilliseconds(1000);
      u8 n_ready = tracking_channels_ready();
      if (n_ready == 0) {
        led_on(LED_GREEN);
        chThdSleepMilliseconds(1000);
        led_off(LED_GREEN);
      } else {
        for (u8 i=0; i<n_ready; i++) {
          led_on(LED_GREEN);
          chThdSleepMilliseconds(250);
          led_off(LED_GREEN);
          chThdSleepMilliseconds(250);
        }
        chThdSleepMilliseconds(1000);
      }
    }
  }
}

/** Sleep thread until a period has elapsed.
 * Keeps track of the previous wake-up time to ensure that a periodic task is
 * woken up on time even if there is jitter in the time the task takes to
 * execute (e.g. due to preemption by higher priority tasks).
 *
 * References:
 *  -# https://www.rfc1149.net/blog/2013/04/03/sleeping-just-the-right-amount-of-time/
 *
 * \param previous Time that the thread was previously woken up
 * \param period Period in system time ticks
 */
void sleep_until(systime_t *previous, systime_t period)
{
  systime_t future = *previous + period;
  chSysLock();
  systime_t now = chVTGetSystemTimeX();
  int must_delay = now < *previous ?
    (now < future && future < *previous) :
    (now < future || future < *previous);
  if (must_delay) {
    chThdSleepS(future - now);
  }
  chSysUnlock();
  *previous = future;
}

static WORKING_AREA_CCM(wa_system_monitor_thread, 1000);
static void system_monitor_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("system monitor");

  systime_t time = chVTGetSystemTime();

  bool ant_status = 0;

  while (TRUE) {

    if (ant_status != frontend_ant_status()) {
      ant_status = frontend_ant_status();
      if (ant_status && frontend_ant_setting() == AUTO) {
        log_info("Now using external antenna.");
      }
      else if (frontend_ant_setting() == AUTO) {
        log_info("Now using patch antenna.");
      }
    }
    u32 status_flags = ant_status << 31 | SBP_MAJOR_VERSION << 16 | SBP_MINOR_VERSION << 8;
    sbp_send_msg(SBP_MSG_HEARTBEAT, sizeof(status_flags), (u8 *)&status_flags);

    /* If we are in base station mode then broadcast our known location. */
    if (broadcast_surveyed_position && position_quality == POSITION_FIX) {
      double tmp[3];
      double base_ecef[3];
      double base_distance;

      llhdeg2rad(base_llh, tmp);
      wgsllh2ecef(tmp, base_ecef);

      vector_subtract(3, base_ecef, position_solution.pos_ecef, tmp);
      base_distance = vector_norm(3, tmp);

      if (base_distance > BASE_STATION_DISTANCE_THRESHOLD) {
        log_warn("Invalid surveyed position coordinates\n");
      } else {
        sbp_send_msg(SBP_MSG_BASE_POS_ECEF, sizeof(msg_base_pos_ecef_t), (u8 *)&base_ecef);
      }
    }

    msg_iar_state_t iar_state;
    if (simulation_enabled_for(SIMULATION_MODE_RTK)) {
      iar_state.num_hyps = 1;
    } else {
      iar_state.num_hyps = dgnss_iar_num_hyps();
    }
    sbp_send_msg(SBP_MSG_IAR_STATE, sizeof(msg_iar_state_t), (u8 *)&iar_state);

    DO_EVERY(2,
     send_thread_states();
    );

    sleep_until(&time, MS2ST(heartbeat_period_milliseconds));
  }
}

static void debug_threads()
{
  const char* state[] = {
    CH_STATE_NAMES
  };
  thread_t *tp = chRegFirstThread();
  while (tp) {
  log_info("%s (%u: %s): prio: %lu, flags: %u, wtobjp: %p",
           tp->p_name, tp->p_state, state[tp->p_state], tp->p_prio,
           tp->p_flags, tp->p_u.wtobjp);
    tp = chRegNextThread(tp);
  }
}

static WORKING_AREA_CCM(wa_watchdog_thread, 1024);
static void watchdog_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("Watchdog");

  /* Allow an extra period at startup since some of the other threads
     take a little while to get going */
  chThdSleepMilliseconds(WATCHDOG_THREAD_PERIOD_MS);

  if (use_wdt)
    wdgStart(&WDGD1, &board_wdg_config);

  while (TRUE) {
    /* Wait for all threads to set a flag indicating they are still
       alive and performing their function */
    chThdSleepMilliseconds(WATCHDOG_THREAD_PERIOD_MS);
    eventmask_t threads_dead = thread_activity_mask
                             ^ chEvtGetAndClearEvents(thread_activity_mask);
    if (threads_dead) {
      /* TODO: ChibiOS thread state dump */
      log_error("One or more threads appear to be dead: 0x%08X. "
                "Watchdog reset %s.",
                (unsigned int)threads_dead,
                use_wdt ? "imminent" : "disabled");
      debug_threads();
    } else {
      if (use_wdt)
        wdgReset(&WDGD1);
    }

  }
}

void system_monitor_setup()
{
  SETTING("system_monitor", "heartbeat_period_milliseconds", heartbeat_period_milliseconds, TYPE_INT);
  SETTING("system_monitor", "watchdog", use_wdt, TYPE_BOOL);

  SETTING("surveyed_position", "broadcast", broadcast_surveyed_position, TYPE_BOOL);
  SETTING("surveyed_position", "surveyed_lat", base_llh[0], TYPE_FLOAT);
  SETTING("surveyed_position", "surveyed_lon", base_llh[1], TYPE_FLOAT);
  SETTING("surveyed_position", "surveyed_alt", base_llh[2], TYPE_FLOAT);


  chThdCreateStatic(
      wa_system_monitor_thread,
      sizeof(wa_system_monitor_thread),
      LOWPRIO+10,
      system_monitor_thread, NULL
  );
  chThdCreateStatic(
      wa_track_status_thread,
      sizeof(wa_track_status_thread),
      LOWPRIO+9,
      track_status_thread, NULL
  );
  watchdog_thread_handle = chThdCreateStatic(
      wa_watchdog_thread,
      sizeof(wa_watchdog_thread),
      HIGHPRIO,
      watchdog_thread, NULL
  );
}

/** Called by each important system thread after doing its important
 * work, to notify the system monitor that it's functioning normally.
 *
 * \param thread_id Unique identifier for the thread.
 **/
void watchdog_notify(watchdog_notify_t thread_id)
{
  chEvtSignal(watchdog_thread_handle, EVENT_MASK(thread_id));
}

/** \} */
