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

#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libswiftnav/sbp.h>

#include "board/leds.h"
#include "board/m25_flash.h"
#include "peripherals/stm_flash.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "sbp.h"
#include "error.h"
#include "flash_callbacks.h"

/** Clock settings for 130.944 MHz from 16.368 MHz HSE. */
const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3 =
{
  .pllm           = 16,
  .plln           = 256,
  .pllp           = 2,
  .pllq           = 6,
  .hpre           = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1          = RCC_CFGR_PPRE_DIV_4,
  .ppre2          = RCC_CFGR_PPRE_DIV_4,
  .flash_config   = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_3WS,
  .apb1_frequency = 32736000,
  .apb2_frequency = 32736000,
};

#define AIRCR_SYSRESETREQ			(1 << 2)
/** Resets the device back into the bootloader. */
void reset_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  /* Ensure all outstanding memory accesses including buffered writes are
   * completed before reset.
   */
  __asm__("DSB;");
  /* Keep priority group unchanged. */
  SCB_AIRCR = AIRCR_VECTKEY |
              AIRCR_PRIGROUP_MASK |
              AIRCR_SYSRESETREQ;
  __asm__("DSB;");
  /* Wait until reset. */
  while(1);
}

/** Register the reset_callback. */
static void reset_callback_register(void)
{
  static sbp_msg_callbacks_node_t reset_node;

  sbp_register_cbk(
    MSG_RESET,
    &reset_callback,
    &reset_node
  );
}

void init(u8 check_fpga_auth)
{
  /* Delay on start-up as some programmers reset the STM twice. */
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  led_setup();

  nap_setup();

  s32 serial_number = nap_conf_rd_serial_number();
  if (serial_number < 0) {
    /* TODO: Handle this properly! */
    serial_number = 0x2222;
  }
  sbp_setup(serial_number);

  /* Check NAP verification status. */
  if (check_fpga_auth) {
    u8 nhs = nap_hash_status();
    if (nhs == NAP_HASH_NOTREADY)
      screaming_death("NAP Verification Failed: Timeout ");
    else if (nhs == NAP_HASH_MISMATCH)
      screaming_death("NAP Verification Failed: Hash mismatch ");
  }

  reset_callback_register();

  flash_callbacks_register();

  stm_unique_id_callback_register();
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

