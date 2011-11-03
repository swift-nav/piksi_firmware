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
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "debug.h"
#include "m25_flash.h"
#include "swift_nap.h"
#include "board/leds.h"
#include "board/exti.h"
#include "board/spi.h"

extern u32 exti_count;
extern u32 data[10][100];

int main(void)
{
	led_setup();
  led_off(LED_GREEN);
  led_off(LED_RED);

  // Debug pins:
  RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  RCC_CR = (RCC_CR & 0x0000FFFF) | RCC_CR_HSEBYP;
  RCC_CR |= RCC_CR_HSEON;

  while (!(RCC_CR & RCC_CR_HSERDY));

//  led_on(LED_GREEN);

  // Clock setup
  RCC_PLLCFGR = (6 << RCC_PLLCFGR_PLLQ_SHIFT) | \
                (1 << RCC_PLLCFGR_PLLP_SHIFT) |  \
                (256 << RCC_PLLCFGR_PLLN_SHIFT) | \
                (16 << RCC_PLLCFGR_PLLM_SHIFT) |
                RCC_PLLCFGR_PLLSRC;

  RCC_CR |= RCC_CR_PLLON;

  while (!(RCC_CR & RCC_CR_PLLRDY));

 // led_on(LED_RED);


  RCC_CFGR = (RCC_CFGR_PPRE_DIV_2 << RCC_CFGR_PPRE1_SHIFT) | \
             (RCC_CFGR_PPRE_DIV_2 << RCC_CFGR_PPRE2_SHIFT) | \
             (RCC_CFGR_HPRE_DIV_2 << RCC_CFGR_HPRE_SHIFT);


  RCC_CFGR = (RCC_CFGR & 0xFFFFFFFC) | (RCC_CFGR_SW_PLL << RCC_CFGR_SW_SHIFT);

  /*while(1) {*/
    /*led_on(LED_RED);*/
    /*for (i = 0; i < 60000; i++)*/
      /*__asm__("nop");*/
    
    /*led_off(LED_RED);*/
    /*for (i = 0; i < 600000; i++)*/
      /*__asm__("nop");*/
  /*}*/

  debug_setup();
  spi_setup();

  printf("Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  swift_nap_setup();

  swift_nap_reset();

  swift_nap_write(0,30,0);
  /*printf("Channel active <= 0\n");*/

	/*printf("Attempting to bring up channel 0...\n");*/
  swift_nap_write(0,9,20); 
  /*printf("Code SVID <= 20\n");*/
  swift_nap_write(0,31,1);
  /*printf("Starting state <= 1 (tracking)\n");*/

  // Set carrier phase rate (carr_pr)
  /*double carr_pr_init = -550.0;*/
  /*double SAMP_FREQ = 16.368e6;*/
  /*u32 CARR_PHASE_FRAC = 24;*/
  /*s32 carr_pr_s32 = (s32)(carr_pr_init*((u32)1 << CARR_PHASE_FRAC)/SAMP_FREQ);*/
  swift_nap_write(0,2,0x0000FDCC);//carr_pr_s32);
  /*printf("Setting carrier phase rate: %f (0x%08X)\n", carr_pr_init, (unsigned int)carr_pr_s32);*/
  
  // Set code phase rate
  /*double code_pr_init = 1.023e6;*/
  /*u32 CODE_PHASE_FRAC = 32;*/
  /*s32 code_pr_s32 = (s32)(code_pr_init*0xFFFFFFFF/SAMP_FREQ);*/
  swift_nap_write(0,12,0x10000000);// code_pr_s32);
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
