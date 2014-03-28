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

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 130944000
#endif

static WORKING_AREA(wa_manage_track_thread, 128);
static msg_t manage_track_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(1000);
    manage_track();
  }

  return 0;
}

static WORKING_AREA(wa_manage_acq_thread, 128);
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

/*
 * Application entry point.
 */
int main(void)
{
  init(1);
  __asm__("CPSID i;");

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

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(wa_manage_track_thread, sizeof(wa_manage_track_thread), NORMALPRIO, manage_track_thread, NULL);
  chThdCreateStatic(wa_manage_acq_thread, sizeof(wa_manage_acq_thread), NORMALPRIO, manage_acq_thread, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * increasing the minutes counter.
   */
  while (TRUE) {
    chThdSleepSeconds(60);
  }
}
