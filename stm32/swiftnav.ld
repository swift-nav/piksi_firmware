/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08010000, LENGTH = 192K /*bootloader is at 0x08000000*/
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

