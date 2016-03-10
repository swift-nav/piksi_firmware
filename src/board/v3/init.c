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

#include <libsbp/sbp.h>

#include <libswiftnav/logging.h>

#include "main.h"
#include "peripherals/leds.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "sbp.h"
#include "error.h"

void init(void)
{
  led_setup();

  nap_setup();

  srand(0);

  s32 serial_number = -1;//nap_conf_rd_serial_number();
  if (serial_number < 0) {
    /* TODO: Handle this properly! */
    serial_number = 0x2222;
  }
  sbp_setup(serial_number);

  fault_handling_setup();

  nap_callbacks_setup();
}

/* Check NAP authentication status. Block and print error message
 * if authentication has failed. This must be done after the NAP,
 * USARTs, and SBP subsystems are set up, so that SBP messages and
 * be sent and received (it can't go in init() or nap_setup()).
 */
void check_nap_auth(void)
{
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

