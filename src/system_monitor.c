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

#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libswiftnav/sbp_messages.h>

#include "board/nap/nap_common.h"
#include "board/leds.h"
#include "main.h"
#include "sbp.h"
#include "sbp_piksi.h"
#include "manage.h"
#include "simulator.h"
#include "system_monitor.h"


/* Time between sending system monitor and heartbeat messages in milliseconds */
uint32_t heartbeat_period_milliseconds = 1000;

/* Base station mode settings. */
/* TODO: Relocate to a different file? */
bool_t base_station_mode = false;
double base_llh[3];

/* Global CPU time accumulator, used to measure thread CPU usage. */
u64 g_ctime = 0;

u32 check_stack_free(Thread *tp)
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
  Thread *tp = chRegFirstThread();
  while (tp) {
    msg_thread_state_t tp_state;
    u16 cpu = 1000.0f * tp->p_ctime / (float)g_ctime;
    tp_state.cpu = cpu;
    tp_state.stack_free = check_stack_free(tp);
    strncpy(tp_state.name, chRegGetThreadName(tp), sizeof(tp_state.name));
    sbp_send_msg(MSG_THREAD_STATE, sizeof(tp_state), (u8 *)&tp_state);

    /* This works because chThdGetTicks is actually a define that pulls out a
     * value from a struct, hopefully if that fact changes then this statement
     * will no longer compile. */
    tp->p_ctime = 0;
    tp = chRegNextThread(tp);
  }
  g_ctime = 0;
}

static WORKING_AREA_CCM(wa_track_status_thread, 128);
static msg_t track_status_thread(void *arg)
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
  return 0;
}

static WORKING_AREA_CCM(wa_system_monitor_thread, 3000);
static msg_t system_monitor_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("system monitor");

  while (TRUE) {
    chThdSleepMilliseconds(heartbeat_period_milliseconds);

    u32 status_flags = 0;
    sbp_send_msg(SBP_HEARTBEAT, sizeof(status_flags), (u8 *)&status_flags);

    /* If we are in base station mode then broadcast our known location. */
    if (base_station_mode) {
      sbp_send_msg(MSG_BASE_POS, sizeof(msg_base_pos_t), (u8 *)&base_llh);
    }

    send_thread_states();

    u32 err = nap_error_rd_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  return 0;
}

void system_monitor_setup()
{
  /* Setup cycle counter for measuring thread CPU time. */
  SCS_DEMCR |= 0x01000000;
  DWT_CYCCNT = 0; /* Reset the counter. */
  DWT_CTRL |= 1 ; /* Enable the counter. */

  SETTING("system_monitor", "heartbeat_period_milliseconds", heartbeat_period_milliseconds, TYPE_INT);

  SETTING("base_station_mode", "enable", base_station_mode, TYPE_BOOL);
  SETTING("base_station_mode", "surveyed lat", base_llh[0], TYPE_FLOAT);
  SETTING("base_station_mode", "surveyed lon", base_llh[1], TYPE_FLOAT);
  SETTING("base_station_mode", "surveyed alt", base_llh[2], TYPE_FLOAT);

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
}

/** \} */

