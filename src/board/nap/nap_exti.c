/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include <ch.h>

#include "nap_exti.h"

#include "../../acq.h"
#include "../../cw.h"
#include "../../track.h"
#include "nap_common.h"
#include "track_channel.h"
#include "../../ext_events.h"
#include "../../system_monitor.h"

/** \addtogroup nap
 * \{ */

/* Number of NAP exti ISR's that have occured.
 * TODO : if this starts being used for anything other than waiting to see
 *        if an exti has occurred, maybe we should change to u64? */
u32 nap_exti_count;

static WORKING_AREA_CCM(wa_nap_exti, 2000);
static msg_t nap_exti_thread(void *arg);
static u32 nap_irq_rd_blocking(void);

static BinarySemaphore nap_exti_sem;

/** Set up NAP GPIO interrupt.
 * Interrupt alerts STM that a channel in NAP needs to be serviced.
 */
void nap_exti_setup(void)
{
  /* Signal from the FPGA is on PA1. */
  chBSemInit(&nap_exti_sem, TRUE);

  /* Enable clock to GPIOA. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
  /* Enable clock to SYSCFG which contains the EXTI functionality. */
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  exti_select_source(EXTI1, GPIOA);
  exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
  exti_reset_request(EXTI1);
  exti_enable_request(EXTI1);

  /* Enable EXTI1 interrupt */
  chThdCreateStatic(wa_nap_exti, sizeof(wa_nap_exti), HIGHPRIO-1, nap_exti_thread, NULL);
  nvicEnableVector(NVIC_EXTI1_IRQ, CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY+2));
}

/** NAP interrupt service routine.
 * Reads the IRQ register from NAP to determine what inside the NAP needs to be
 * serviced, and then calls the appropriate service routine.
 */
void exti1_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  exti_reset_request(EXTI1);

  /* Wake up processing thread */
  chBSemSignalI(&nap_exti_sem);

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}


static void handle_nap_exti(void)
{
  /* XXX Including ch.h in nap_common.h for an extern declaration causes
   * serious breakage. */
  extern BinarySemaphore timing_strobe_sem;

  u32 irq = nap_irq_rd_blocking();

  if (irq & NAP_IRQ_ACQ_DONE)
    acq_service_irq();

  if (irq & NAP_IRQ_ACQ_LOAD_DONE)
    acq_service_load_done();

  if (irq & NAP_IRQ_TIMING_STROBE) {
    chBSemReset(&timing_strobe_sem, TRUE);
  }

  if (irq & NAP_IRQ_EXT_EVENT)
    ext_event_service();

  /* Mask off everything but tracking irqs. */
  irq &= NAP_IRQ_TRACK_MASK;

  /* Loop over tracking irq bit flags. */
  for (u8 n = 0; n < nap_track_n_channels; n++) {
    /* Save a bit of time by seeing if the rest of the bits
     * are zero in one go so we don't have to loop over all
     * of them.
     */
    if (!(irq >> n))
      break;

    /* Test if the nth tracking irq flag is set, if so service it. */
    if ((irq >> n) & 1) {
      tracking_channel_update(n);
    }
  }

  watchdog_notify(WD_NOTIFY_NAP_ISR);
  nap_exti_count++;
}

static msg_t nap_exti_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("NAP ISR");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWait(&nap_exti_sem);

    /* We need a level (not edge) sensitive interrupt -
     * if there is another interrupt pending on the Swift
     * NAP then the IRQ line will stay high. Therefore if
     * the line is still high, don't suspend the thread.
     */
    while (GPIOA_IDR & GPIO1) {
      handle_nap_exti();
    }

  }
  return 0;
}

/** Get number of NAP ISR's that have occurred.
 *
 * \return Latest NAP ISR count.
 */
u32 last_nap_exti_count(void)
{
  return nap_exti_count;
}

/** Wait until next NAP ISR has occurred. */
void wait_for_nap_exti(void)
{
  u32 last_last_exti = last_nap_exti_count();

  while (last_nap_exti_count() == last_last_exti) ;
}

/** Read NAP's IRQ register.
 * NAP's IRQ register shows which channels in NAP need to be serviced. NAP IRQ
 * line will stay high as long as any bit in IRQ register reads is high.
 *
 * \return 32 bit value from NAP's IRQ register.
 */
static u32 nap_irq_rd_blocking(void)
{
  u8 temp[4] = { 0, 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_IRQ, 4, temp, temp);
  return (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];
}

/** \} */

