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

#include <libsbp/flash.h>
#include <libsbp/sbp.h>

#include <libswiftnav/logging.h>

#include "main.h"
#include "peripherals/leds.h"
#include "peripherals/random.h"
#include "board/v2/m25_flash.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "sbp.h"
#include "error.h"

#define SCB_AIRCR_VECTKEY                       (0x5FA << SCB_AIRCR_VECTKEY_Pos)
#define SCB_AIRCR_SYSRESETREQ                   (1 << 2)

/** Resets the device back into the bootloader. */
static void reset_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  /* Ensure all outstanding memory accesses including buffered writes are
   * completed before reset.
   */
  __asm__("DSB;");
  /* Keep priority group unchanged. */
  SCB->AIRCR = SCB_AIRCR_VECTKEY |
               SCB_AIRCR_PRIGROUP_Msk |
               SCB_AIRCR_SYSRESETREQ;
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

#define STM_UNIQUE_ID_ADDR 0x1FFF7A10
/** Callback to read STM32F4's hardcoded unique ID.
 * Sends STM32F4 unique ID (12 bytes) back to host.
 */
static void stm_unique_id_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  sbp_send_msg(SBP_MSG_STM_UNIQUE_ID_RESP, 12, (u8*)STM_UNIQUE_ID_ADDR);
}

/** Register callback to read Device's Unique ID. */
static void stm_unique_id_callback_register(void)
{
  static sbp_msg_callbacks_node_t stm_unique_id_node;

  sbp_register_cbk(SBP_MSG_STM_UNIQUE_ID_REQ,
                   &stm_unique_id_callback,
                   &stm_unique_id_node);
}

void init(void)
{
  /* Delay on start-up as some programmers reset the STM twice. */
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  led_setup();

  nap_setup();

  usarts_setup();
  s32 serial_number = nap_conf_rd_serial_number();
  if (serial_number < 0) {
    /* TODO: Handle this properly! */
    serial_number = 0x2222;
  }
  sbp_setup(serial_number);

  rng_setup();
  srand(random_int());

  fault_handling_setup();

  nap_callbacks_setup();

  reset_callback_register();

  stm_unique_id_callback_register();
}

/* Check NAP authentication status. Block and print error message
 * if authentication has failed. This must be done after the NAP,
 * USARTs, and SBP subsystems are set up, so that SBP messages and
 * be sent and received (it can't go in init() or nap_setup()).
 */
void check_nap_auth(void)
{
  u8 nhs = nap_hash_status();
  if (nhs != NAP_HASH_MATCH) {
    led_on(LED_GREEN);
    led_off(LED_RED);
    while (1) {
      DO_EVERY(10000000,
        log_error("NAP Verification Failed");
        led_toggle(LED_GREEN);
        led_toggle(LED_RED);
      );
    }
  }
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

