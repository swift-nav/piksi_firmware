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
#include <string.h>
#include "ch.h"

#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>

#include "../src/board/leds.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"

#include "../src/sbp.h"
#include "../src/init.h"
#include "../src/manage.h"
#include "../src/track.h"
#include "../src/timing.h"
#include "../src/position.h"

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 130944000
#endif

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
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */
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

static WORKING_AREA(wa_sbp_thread, 4096);
static msg_t sbp_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(50);
    sbp_process_messages();
  }

  return 0;
}

/* TODO: Think about thread safety when updating ephemerides. */
ephemeris_t es[32];
ephemeris_t es_old[32];

static WORKING_AREA(wa_nav_msg_thread, 4096);
static msg_t nav_msg_thread(void *arg)
{
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(3000);

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    /* TODO: This should be trigged by a semaphore from the tracking loop, not
     * just ran periodically. */

    memcpy(es_old, es, sizeof(es));
    for (u8 i=0; i<nap_track_n_channels; i++)
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        s8 ret = process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
        if (ret < 0)
          printf("PRN %02d ret %d\n", tracking_channel[i].prn+1, ret);

        if (ret == 1 && !es[tracking_channel[i].prn].healthy)
          printf("PRN %02d unhealthy\n", tracking_channel[i].prn+1);
        if (memcmp(&es[tracking_channel[i].prn], &es_old[tracking_channel[i].prn], sizeof(ephemeris_t))) {
          printf("New ephemeris for PRN %02d\n", tracking_channel[i].prn+1);
          /* TODO: This is a janky way to set the time... */
          gps_time_t t;
          t.wn = es[tracking_channel[i].prn].toe.wn;
          t.tow = tracking_channel[i].TOW_ms / 1000.0;
          if (gpsdifftime(t, es[tracking_channel[i].prn].toe) > 2*24*3600)
            t.wn--;
          else if (gpsdifftime(t, es[tracking_channel[i].prn].toe) < 2*24*3600)
            t.wn++;
          /*set_time(TIME_COARSE, t);*/
        }
        /*if (es[tracking_channel[i].prn].valid == 1) {*/
          /*sendrtcmnav(&es[tracking_channel[i].prn], tracking_channel[i].prn);*/
        /*}*/
      }
  }

  return 0;
}

static Thread *tp = NULL;
#define tim5_isr Vector108
void tim5_isr()
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  /* Wake up processing thread */
  if (tp != NULL) {
    chSchReadyI(tp);
    tp = NULL;
  }

  timer_clear_flag(TIM5, TIM_SR_UIF);

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

static WORKING_AREA(wa_solution_thread, 8000);
static msg_t solution_thread(void *arg)
{
  (void)arg;

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chSysLock();
    tp = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    chSysUnlock();

    led_toggle(LED_RED);
  }
  return 0;
}

void soln_timer_setup()
{
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvicEnableVector(NVIC_TIM5_IRQ, CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+1));
  timer_reset(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, 0);
  timer_disable_preload(TIM5);
  timer_set_period(TIM5, 65472000); /* 1 second. */
  timer_enable_counter(TIM5);
  timer_enable_irq(TIM5, TIM_DIER_UIE);
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
  soln_timer_setup();

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(wa_manage_track_thread, sizeof(wa_manage_track_thread), NORMALPRIO-2, manage_track_thread, NULL);
  chThdCreateStatic(wa_nav_msg_thread, sizeof(wa_nav_msg_thread), NORMALPRIO-1, nav_msg_thread, NULL);
  chThdCreateStatic(wa_manage_acq_thread, sizeof(wa_manage_acq_thread), NORMALPRIO, manage_acq_thread, NULL);
  chThdCreateStatic(wa_nap_error_thread, sizeof(wa_nap_error_thread), NORMALPRIO-22, nap_error_thread, NULL);
  chThdCreateStatic(wa_sbp_thread, sizeof(wa_sbp_thread), HIGHPRIO-22, sbp_thread, NULL);
  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread), HIGHPRIO-2, solution_thread, NULL);

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
