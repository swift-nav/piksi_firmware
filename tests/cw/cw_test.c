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
#include <math.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "cw.h"
#include "manage.h"
#include "hw/leds.h"
#include "hw/spi.h"
#include "hw/m25_flash.h"

int main(void)
{
  init();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  led_toggle(LED_GREEN);
  led_toggle(LED_RED);

	u64 cw_power;
	float cw_freq;

	while(1) {

		printf("#PLOT_DATA_START\n");

		// Load CW ram
		cw_schedule_load(timing_count() + 1000);
		while (!(cw_get_load_done()));
		printf("# Finished loading cw ram\n");

		// Do CW detection
//		cw_start(-4e6,4e6,8e6/(SPECTRUM_LEN-1));
//		cw_start(0.5e6,0.7e6,0.2e6/(SPECTRUM_LEN-1));
    float cf = (1.575542*1e9-1.575420*1e9);
    float span = 200e3;
		cw_start(cf-span/2,cf+span/2,span/(SPECTRUM_LEN-1));
		while (!(cw_get_running_done()));
		printf("# Finished doing cw detection\n");

		for (u16 si=0;si<SPECTRUM_LEN;si++) {

			for (u32 dly = 0; dly < 50000; dly++)
				__asm__("nop");

			cw_get_spectrum_point(&cw_freq,&cw_power,si);

      if (~((cw_power == 0) && (cw_freq == 0))) {
//			printf("%+7.2f %lu # %d\n",cw_freq,(long unsigned int)cw_power,(unsigned int)si);
			printf("%+7.1f %lu\n",cw_freq,(long unsigned int)cw_power);
//			printf("%+4.2f %lu\n",cw_freq,(long unsigned int)cw_power);
      }

			u32 err = swift_nap_read_error_blocking();
			if (err) {
				printf("Error: 0x%08X\n", (unsigned int)err);
				while(1);
			}
		}

		printf("#PLOT_DATA_END\n");

	}

  while (1);

	return 0;
}

