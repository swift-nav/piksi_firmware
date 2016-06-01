/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

#include <stdlib.h>
#include <string.h>

#include <libsbp/sbp.h>

#include <libswiftnav/logging.h>

#include "main.h"
#include "peripherals/leds.h"
#include "board/nap/nap_common.h"
#include "nap/nap_conf.h"
#include "sbp.h"
#include "error.h"

#define REQUIRED_NAP_VERSION_MASK (0xFFFF0000U)
#define REQUIRED_NAP_VERSION_VAL  (0x03000000U)

#define SLCR_PSS_RST_CTRL (*(volatile u32 *)0xf8000200)
#define SLCR_PSS_RST_CTRL_SOFT_RST 1

#define REG_TEST_THD_COUNT    4
#define REG_TEST_STACK_SIZE   2048
#define REG_TEST_PRIO_N(n)    (reg_test_prios[(n)])
#define REG_TEST_WA_SIZE_B    THD_WORKING_AREA_SIZE(REG_TEST_STACK_SIZE)
#define REG_TEST_WA_SIZE_SA   (REG_TEST_WA_SIZE_B / sizeof(stkalign_t))
#define REG_TEST_WA_N(n)      (&reg_test_was[(n) * REG_TEST_WA_SIZE_SA])

static const u8 reg_test_prios[REG_TEST_THD_COUNT] = {
  LOWPRIO, NORMALPRIO-3, HIGHPRIO-2, HIGHPRIO-1
};
static stkalign_t reg_test_was[REG_TEST_THD_COUNT * REG_TEST_WA_SIZE_SA];
extern THD_FUNCTION(reg_test_thread, arg);
static void reg_test_setup(void);

static bool nap_version_ok(u32 version);
static void nap_version_check(void);
static void nap_auth_check(void);

/** Resets the device back into the bootloader. */
static void reset_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  /* Ensure all outstanding memory accesses including buffered writes are
   * completed before reset.
   */
  __asm__("DSB;");
  SLCR_PSS_RST_CTRL = SLCR_PSS_RST_CTRL_SOFT_RST;
  __asm__("DSB;");
  /* Wait until reset. */
  while(1);
}

/** Register the reset_callback. */
static void reset_callback_register(void)
{
  static sbp_msg_callbacks_node_t reset_node;

  sbp_register_cbk(
    SBP_MSG_RESET,
    &reset_callback,
    &reset_node
  );
}

void pre_init(void)
{
  led_setup();
}

void init(void)
{
  fault_handling_setup();
  reset_callback_register();

  nap_version_check();
  nap_setup();
  nap_auth_check();
  nap_callbacks_setup();

  srand(0);

  reg_test_setup();
}

static void reg_test_setup(void)
{
  for (u32 i=0; i<REG_TEST_THD_COUNT; i++) {
    thread_t *thd =
        chThdCreateStatic(REG_TEST_WA_N(i), REG_TEST_WA_SIZE_B,
                          REG_TEST_PRIO_N(i), reg_test_thread, NULL);
    chRegSetThreadNameX(thd, "reg test");
  }
}

static bool nap_version_ok(u32 version)
{
  return ((version & REQUIRED_NAP_VERSION_MASK) == REQUIRED_NAP_VERSION_VAL);
}

static void nap_version_check(void)
{
  u32 nap_version = nap_conf_rd_version();
  if (!nap_version_ok(nap_version)) {
    while (1) {
      log_error("Unsupported NAP version: 0x%08x", nap_version);
      chThdSleepSeconds(2);
    }
  }
}

/* Check NAP authentication status. Block and print error message
 * if authentication has failed. This must be done after the NAP,
 * USARTs, and SBP subsystems are set up, so that SBP messages and
 * be sent and received (it can't go in init() or nap_setup()).
 */
static void nap_auth_check(void)
{

}

s32 serial_number_get(void)
{
  /* TODO: read from NVM */
  return -1;
}

u8 hw_revision_string_get(char *hw_revision_string)
{
  /* TODO: read from NVM */
  const char *s = "alpha";
  strcpy(hw_revision_string, s);
  return strlen(hw_revision_string);
}

u8 nap_version_string_get(char *nap_version_string)
{
  return nap_conf_rd_version_string(nap_version_string);
}

/** Our own basic implementation of sbrk().
 * This overrides the version provided by newlib/libnosys which now checks that
 * the heap_end pointer doesn't grow pass the stack pointer. Thats great except
 * on the STM32F4 we are putting our stack in the CCM memory region which has
 * an address lower than the main RAM region (where the heap is) causing the
 * default sbrk() to always return ENOMEM.
 *
 * \todo Re-implement stack pointer checking taking into account our memory
 *       layout.
 */
void *_sbrk (int incr)
{
  extern char   end; /* Set by linker.  */
  static char * heap_end;
  char *        prev_heap_end;

  if (heap_end == 0)
    heap_end = & end;

  prev_heap_end = heap_end;

  heap_end += incr;

  return (void *)prev_heap_end;
}

