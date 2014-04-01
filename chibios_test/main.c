/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdio.h>
#include "ch.h"

#include "../src/board/leds.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"

#include "../src/sbp.h"
#include "../src/init.h"
#include "../src/manage.h"
#include "../src/track.h"

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 130944000
#endif

size_t get_thd_free_stack(void *wsp, size_t size)
{
  size_t n = 0;
#if CH_DBG_FILL_THREADS
  uint8_t *startp = (uint8_t *)wsp + sizeof(Thread);
  uint8_t *endp = (uint8_t *)wsp + size;
  while (startp < endp)
    if(*startp++ == 0x55) ++n;
#endif
  return n;
}

static WORKING_AREA(wa_manage_track_thread, 4096);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(200);
    manage_track();
    tracking_send_state();
  }

  return 0;
}

static WORKING_AREA(wa_manage_acq_thread, 4096);
static msg_t manage_acq_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    led_toggle(LED_GREEN);
    chThdSleepMilliseconds(100);
    manage_acq();
  }

  return 0;
}

static WORKING_AREA(wa_nap_error_thread, 1024);
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

extern u8 __main_stack_base__;
extern u8 __process_stack_base__;
/*
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
/*
 * Application entry point.
 */
int main(void)
{
  /**
   * Hardware initialization, in this simple demo just the systick timer is
   * initialized.
   */
  STBase->RVR = SYSTEM_CLOCK / CH_FREQUENCY - 1;
  STBase->CVR = 0;
  STBase->CSR = CLKSOURCE_CORE_BITS | ENABLE_ON_BITS | TICKINT_ENABLED_BITS;

  /*
   * System initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  chSysInit();
  init(1);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  u8 nap_git_hash[20];
  nap_conf_rd_git_hash(nap_git_hash);
  printf("SwiftNAP git: ");
  for (u8 i=0; i<20; i++)
    printf("%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    printf(" (unclean)");
  printf("\n");
  printf("SwiftNAP configured with %d tracking channels\n\n", nap_track_n_channels);

  manage_acq_setup();
  timing_setup();
  position_setup();

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(wa_manage_track_thread, sizeof(wa_manage_track_thread), NORMALPRIO, manage_track_thread, NULL);
  chThdCreateStatic(wa_manage_acq_thread, sizeof(wa_manage_acq_thread), NORMALPRIO, manage_acq_thread, NULL);
  chThdCreateStatic(wa_nap_error_thread, sizeof(wa_nap_error_thread), NORMALPRIO, nap_error_thread, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * increasing the minutes counter.
   */
  while (TRUE) {
    chThdSleepSeconds(1);
    /*debug_threads();*/
    /*
    printf("main: %d\nproc: %d\nma: %d\nmt: %d\nne: %d\nexti: \n",
        get_thd_free_stack(&__main_stack_base__, 0x400),
        get_thd_free_stack(&__process_stack_base__, 0x1000),
        get_thd_free_stack(wa_manage_acq_thread, 1024),
        get_thd_free_stack(wa_manage_track_thread, 1024),
        get_thd_free_stack(wa_nap_error_thread, 1024));
    */
  }
}
