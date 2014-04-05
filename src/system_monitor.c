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

#include "board/nap/nap_common.h"
#include "main.h"
#include "sbp.h"
#include "sbp_piksi.h"
#include "system_monitor.h"

/* Global CPU time accumulator, used to measure thread CPU usage. */
u64 g_ctime = 0;

void send_thread_states()
{
  Thread *tp = chRegFirstThread();
  while (tp) {
    msg_thread_state_t tp_state;
    u16 cpu = 1000.0f * tp->p_ctime / (float)g_ctime;
    tp_state.cpu = cpu;
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

static WORKING_AREA(wa_nap_error_thread, 4096);
static msg_t nap_error_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("system monitor");

  while (TRUE) {
    chThdSleepMilliseconds(500);
    DO_EVERY(2,
        sbp_send_msg(MSG_HEARTBEAT, 0, 0);
        send_thread_states();
    );
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

  chThdCreateStatic(
      wa_nap_error_thread,
      sizeof(wa_nap_error_thread),
      LOWPRIO+10,
      nap_error_thread, NULL
  );
}

/** \} */

