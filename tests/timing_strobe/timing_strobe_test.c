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

#include <stdio.h>

#include "init.h"
#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "acq.h"
#include "hw/leds.h"
#include "hw/m25_flash.h"

int main(void)
{

  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- TIMING STROBE TEST ---\n\r");

  u32 tcs;
  u32 tcls;
  u32 strobe_offset = 2000;

  tcs = timing_count();
  acq_schedule_load(tcs + strobe_offset);
  tcls = timing_count_latched();

  printf("timing_count[%d]          = %u\n", 0, (unsigned int)tcs);
  printf("timing_count_latched[%d]  = %u\n", 0, (unsigned int)tcls);
  printf("difference = %u\n", (unsigned int)(tcls-tcs));

  led_off(LED_GREEN);
  led_on(LED_RED);

  while(1);

	return 0;
}

