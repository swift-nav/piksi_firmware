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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/dma.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "manage.h"
#include "hw/leds.h"
#include "hw/spi.h"
#include "hw/m25_flash.h"

#include <swiftlib/pvt.h>
#include <swiftlib/ephemeris.h>
#include <swiftlib/coord_system.h>
#include <swiftlib/linear_algebra.h>

const clock_scale_t hse_16_368MHz_in_65_472MHz_out_3v3 =
{ /* 65.472 MHz */
  .pllm = 16,
  .plln = 256,
  .pllp = 4,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_2WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 16368000,
};

const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3 =
{ /* 130.944 MHz (Overclocked!!) */
  .pllm = 16,
  .plln = 256,
  .pllp = 2,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_8,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 2*16368000,
};

const clock_scale_t hse_16_368MHz_in_120_203MHz_out_3v3 =
{ /* 120.203 MHz (USB OK but APB freq */
  .pllm = 16,
  .plln = 235,
  .pllp = 2,
  .pllq = 5,
  .hpre = RCC_CFGR_HPRE_DIV_2,
  .ppre1 = RCC_CFGR_PPRE_DIV_2,
  .ppre2 = RCC_CFGR_PPRE_DIV_2,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 30050625,
  .apb2_frequency = 30050625,
};

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  // Debug pins (CC1111 TX/RX)
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);

  debug_setup();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");


  swift_nap_setup();
  swift_nap_reset();

  m25_setup();

  manage_acq_setup();
 
  led_toggle(LED_RED);

  const double WPR_llh[3] = {D2R*37.038350, D2R*-122.141812, 376.7};

  double WPR_xyz[3];
  wgsllh2xyz(WPR_llh, WPR_xyz);
  
  static ephemeris_t es[32];
  static gnss_satellite_state sat_states[32];
  while(1)
  {
    for (u32 i = 0; i < 3000; i++)
      __asm__("nop");
    debug_process_messages();
    manage_track();
    manage_acq();

    // Check if there is a new nav msg subframe to process.
    // TODO: move this into a function
    
    for (u8 i=0; i<TRACK_N_CHANNELS; i++)
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        printf(" PRN %d",tracking_channel[i].prn + 1);
        process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
      }

    u8 n_ready = 0;
    for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
      if (es[tracking_channel[i].prn].valid == 1 && es[tracking_channel[i].prn].healthy == 1 && tracking_channel[i].state == TRACKING_RUNNING)
        n_ready++;
    }
    if (n_ready >= 4) {
      /* Got enough sats/ephemerides, do a solution. */
      printf("Starting solution!\n");
      double prs[TRACK_N_CHANNELS];
      double prrs[TRACK_N_CHANNELS];
      double TOTs[TRACK_N_CHANNELS];
      double ranges[TRACK_N_CHANNELS];
      double mean_range=0;
      calc_pseudoranges(prs, prrs, TOTs);
      for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
        sat_states[i].prn = tracking_channel[i].prn;
        sat_states[i].recv_idx = 0;
        calc_sat_pos(sat_states[i].pos,
                     sat_states[i].vel,
                     &sat_states[i].clock_err,
                     &sat_states[i].clock_rate_err,
                     &es[sat_states[i].prn],
                     TOTs[i]);
        sat_states[i].pseudorange = prs[i] + sat_states[i].clock_err*NAV_C;
        sat_states[i].pseudorange_rate = prrs[i] - sat_states[i].clock_rate_err*NAV_C;
        ranges[i] = predict_range(WPR_xyz, TOTs[i], &es[sat_states[i].prn]);
        mean_range += ranges[i];
      }
      mean_range /= TRACK_N_CHANNELS;
      for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
        double r = ranges[i] - mean_range + NOMINAL_RANGE;
        printf("PRN: %d, ", sat_states[i].prn+1);
        printf("tow: %f, ", (double)tracking_channel[i].TOW_ms);
        printf("TOT: %f, ", TOTs[i]);
        printf("err: %f, ", sat_states[i].clock_err*NAV_C);
        printf("r: %f, ", ranges[i]);
        printf("ppr: %f, ", r);
        printf("pr: %f, ", sat_states[i].pseudorange);
        printf("prr: %f, \n", sat_states[i].pseudorange_rate);
      }

      printf("pr = array([");
      for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
        printf("%f, ", sat_states[i].pseudorange);
      }
      printf("])\n");
      printf("ppr = array([");
      for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
        double r = ranges[i] - mean_range + NOMINAL_RANGE;
        printf("%f, ", r);
      }
      printf("])\n");
      printf("pr_err = array([");
      for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
        double r = ranges[i] - mean_range + NOMINAL_RANGE;
        printf("%f, ", (sat_states[i].pseudorange - r)*1023/3e5);
      }
      printf("])\n");


      gnss_solution soln;
      solution_plus plus;
      double W[32] = EQUAL_WEIGHTING;
      calc_PVT(&soln, 4, 1, sat_states, W, &plus);
      double temp[3];
      vector_subtract(soln.pos_xyz, WPR_xyz, temp);
      double dist = vector_norm(temp);
      printf("======= SOLUTION ==============================\n");
      printf("Lat: %f, Lon: %f, Height: %f\n", R2D*soln.pos_llh[0], R2D*soln.pos_llh[1], soln.pos_llh[2]);
      printf("X: %f, Y: %f, Z: %f\n", soln.pos_xyz[0], soln.pos_xyz[1], soln.pos_xyz[2]);
      printf("Dist to WPR: %f\n", dist);
      printf("Err Cov: %f %f %f %f %f %f\n", soln.err_cov[0], soln.err_cov[1], soln.err_cov[2], soln.err_cov[3], soln.err_cov[4], soln.err_cov[5]);
      printf("PDOP: %f \n", plus.pdop);
      printf("===============================================\n");
      while(1);
    }

    DO_EVERY(500,
      float snrs[TRACK_N_CHANNELS];
      for (u8 i=0; i<TRACK_N_CHANNELS; i++)
        if (tracking_channel[i].state == TRACKING_RUNNING)
          snrs[i] = tracking_channel_snr(i);
        else
          snrs[i] = -1.0;
      debug_send_msg(0x22, sizeof(snrs), (u8*)snrs);
    );
    u32 err = swift_nap_read_error_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);

  }

  while (1);
  
	return 0;
}

