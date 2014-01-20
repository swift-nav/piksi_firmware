/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Define memory regions. */
MEMORY
{
  /* Bootloader occupies first sector of flash. */
	rom (rx) : ORIGIN = 0x08004000, LENGTH = 240K
	coffee_fs_rom (r) : ORIGIN = 0x08040000, LENGTH = 768K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  /* STM32F4 Core Coupled Memory, not accesible by peripherals or DMA. */
  ccm (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f4.ld

SECTIONS {
  _stack_size = 65536;
  _stack = ORIGIN(ccm)+LENGTH(ccm);
  _stack_begin = _stack - _stack_size;

  .settings_area : {
    _settings_area = .;
    *(.settings_area*)
    _esettings_area = .;
  } > rom

}

PROVIDE(_coffee_fs_area = ORIGIN(coffee_fs_rom));
PROVIDE(_ecoffee_fs_area = ORIGIN(coffee_fs_rom) + LENGTH(coffee_fs_rom));

