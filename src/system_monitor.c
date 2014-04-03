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

/*
extern u8 __main_stack_base__;
extern u8 __process_stack_base__;
*/

void send_thread_states()
{
  Thread *tp = chRegFirstThread();
  while (tp) {
    msg_thread_state_t tp_state;
    tp_state.cpu = chThdGetTicks(tp);
    strncpy(tp_state.name, chRegGetThreadName(tp), sizeof(tp_state.name));
    sbp_send_msg(MSG_THREAD_STATE, sizeof(tp_state), (u8 *)&tp_state);

    /* This works because chThdGetTicks is actually a define that pulls out a
     * value from a struct, hopefully if that fact changes then this statement
     * will no longer compile. */
    chThdGetTicks(tp) = 0;
    tp = chRegNextThread(tp);
  }
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
  chThdCreateStatic(
      wa_nap_error_thread,
      sizeof(wa_nap_error_thread),
      LOWPRIO+10,
      nap_error_thread, NULL
  );
}

/** \} */

