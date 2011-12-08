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
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "debug.h"
#include "max2769.h"
#include "m25_flash.h"
#include "swift_nap.h"
#include "board/leds.h"
#include "board/exti.h"
#include "board/spi.h"

extern u32 exti_count;
extern u32 data[10][100];

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
  .hpre = RCC_CFGR_HPRE_DIV_2,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 16368000,
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
	led_setup();
  led_off(LED_GREEN);
  led_off(LED_RED);

  // Debug pins:
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);
  /*rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);*/

  debug_setup();
  spi_setup();
  timer_setup();

  printf("Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  while(1) {
    led_toggle(LED_RED);

    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }

  swift_nap_setup();

  swift_nap_reset();

  swift_nap_write(0,30,0);
  printf("Channel active <= 0\n");

  swift_nap_write(0,9,20); 
  printf("Code SVID <= 20\n");

  swift_nap_write(0,3,(u32)-1050); // Acq initial carrier freq
  swift_nap_write(0,4,500); // Acq carrier freq increment
  swift_nap_write(0,5,3); // Acq carrier freq loops

  swift_nap_write(0,29,0); // Disable granular acq
  swift_nap_write(0,31,0); // Starting state = acq

  swift_nap_write(0,13,0); // Acq starting code phase (10 MSBs)
  swift_nap_write(0,14,0); // Acq starting code phase (32 LSBs)
  swift_nap_write(0,15,819); // Num code phase loops (819 ~ whole search space)

  swift_nap_write(0,16,65472); // Acq integration window size (samples) - 65472 = 4ms

  swift_nap_write(0,30,1);
  printf("Channel active <= 1\n");

  /*swift_nap_write(0,31,1);*/
  /*printf("Starting state <= 1 (tracking)\n");*/

  // Set carrier phase rate (carr_pr)
  /*double carr_pr_init = -550.0;*/
  /*double SAMP_FREQ = 16.368e6;*/
  /*u32 CARR_PHASE_FRAC = 24;*/
  /*s32 carr_pr_s32 = (s32)(carr_pr_init*((u32)1 << CARR_PHASE_FRAC)/SAMP_FREQ);*/
  /*swift_nap_write(0,2,0x0000FDCC);//carr_pr_s32);*/
  /*printf("Setting carrier phase rate: %f (0x%08X)\n", carr_pr_init, (unsigned int)carr_pr_s32);*/
  
  // Set code phase rate
  /*double code_pr_init = 1.023e6;*/
  /*u32 CODE_PHASE_FRAC = 32;*/
  /*s32 code_pr_s32 = (s32)(code_pr_init*0xFFFFFFFF/SAMP_FREQ);*/
  /*swift_nap_write(0,12,0x10000000);// code_pr_s32);*/
  /*printf("Setting code phase rate: %f (0x%08X)\n", code_pr_init, (unsigned int)code_pr_s32);*/

  gpio_set(GPIOC, GPIO10);
  exti_setup();


  for (u32 i = 0; i < 6000; i++)
    __asm__("nop");
  /*printf("Channel active <= 1\n");*/
  swift_nap_write(0,30,1);

  while (1)
  {
    if (exti_count >= 100) {
      printf("----------------------------------------------\n");
      for (u32 i=0; i<exti_count; i++) {
        for (u8 j=0; j<10; j++)
          printf("% 6i\t", (int)data[j][i]);
        printf("\n");
      }
      printf("----------------------------------------------\n");
      /*swift_nap_write(0,30,0);*/
      /*exti_reset_request(EXTI0);*/
      break;
      /*exti_count = 0;*/
    }

    
    /*led_toggle(LED_RED);*/

    /*for (u32 i = 0; i < 60000; i++)*/
      /*__asm__("nop");*/
	}
  gpio_clear(GPIOC, GPIO10);
  while(1) {
    led_toggle(LED_RED);

    for (u32 i = 0; i < 60000; i++)
      __asm__("nop");
  }

	return 0;
}
