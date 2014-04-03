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

#include <ch.h>

#include "system_monitor.h"

/*
extern u8 __main_stack_base__;
extern u8 __process_stack_base__;

void debug_threads()
{
  printf("t = %lu\nThreads:\n", chTimeNow());
  Thread *tp = chRegFirstThread();
  while (tp) {
    printf("\t%s: %lu (%.1f)"
        chRegGetThreadName(),
        chThdGetTicks(),
        chThdGetTicks() / (float)chTimeNow()
    );
    tp = chRegNextThread(tp);
  }
}
*/

static WORKING_AREA(wa_nap_error_thread, 4096);
static msg_t nap_error_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(500);
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

