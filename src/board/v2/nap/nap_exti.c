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


#include <ch.h>
#include <hal.h>

#include "nap_exti.h"

#include "nap_acq.h"
#include "../../cw.h"
#include "../../track.h"
#include "nap_common.h"
#include "track_channel.h"
#include "../../ext_events.h"
#include "../../system_monitor.h"
#include "peripherals/spi_wrapper.h"

/** \addtogroup nap
 * \{ */

#define PROCESS_PERIOD_ms (1000)

/* Number of NAP exti ISR's that have occured.
 * TODO : if this starts being used for anything other than waiting to see
 *        if an exti has occurred, maybe we should change to u64? */
u32 nap_exti_count;

static WORKING_AREA_CCM(wa_nap_exti, 2000);
static void nap_exti_thread(void *arg);
static u32 nap_irq_rd_blocking(void);

void exti1_isr(EXTDriver *, expchannel_t);

static BSEMAPHORE_DECL(nap_exti_sem, TRUE);
static EXTConfig extconfig  = {
  .channels[1] = {
    .mode = EXT_MODE_GPIOA | EXT_CH_MODE_RISING_EDGE,
    .cb = exti1_isr,
  },
};

/** Set up NAP GPIO interrupt.
 * Interrupt alerts STM that a channel in NAP needs to be serviced.
 */
void nap_exti_setup(void)
{
  /* Signal from the FPGA is on PA1. */

  extStart(&EXTD1, &extconfig);
  extChannelEnable(&EXTD1, 1);

  /* Enable EXTI1 interrupt */
  chThdCreateStatic(wa_nap_exti, sizeof(wa_nap_exti), HIGHPRIO-1, nap_exti_thread, NULL);
}

/** NAP interrupt service routine.
 * Reads the IRQ register from NAP to determine what inside the NAP needs to be
 * serviced, and then calls the appropriate service routine.
 */
void exti1_isr(EXTDriver *driver, expchannel_t channel)
{
  (void)driver; (void)channel;
  chSysLockFromISR();


  /* Wake up processing thread */
  chBSemSignalI(&nap_exti_sem);

  chSysUnlockFromISR();
}


static void handle_nap_exti(void)
{
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
  tracking_channels_update(irq);

  u32 err = nap_error_rd_blocking();
  if (err) {
    log_error("SwiftNAP Error: 0x%08X", (unsigned int)err);
    tracking_channels_missed_update_error(err);
  }

  watchdog_notify(WD_NOTIFY_NAP_ISR);
  nap_exti_count++;
}

static void nap_exti_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("NAP ISR");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_exti_sem, MS2ST(PROCESS_PERIOD_ms));

    /* We need a level (not edge) sensitive interrupt -
     * if there is another interrupt pending on the Swift
     * NAP then the IRQ line will stay high. Therefore if
     * the line is still high, don't suspend the thread.
     */

    spi_lock(SPI_SLAVE_FPGA);
    while (palReadLine(LINE_NAP_IRQ)) {
      handle_nap_exti();
    }
    tracking_channels_process();
    spi_unlock(SPI_SLAVE_FPGA);

  }
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

