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
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();
  led_off(LED_GREEN);
  led_off(LED_RED);

  // Debug pins (CC1111 TX/RX)
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);
  /*rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);*/

  debug_setup();
  spi_setup();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  swift_nap_setup();
  exti_setup();


  /*u8 foo[4] = {0xDE, 0xAD, 0xBE, 0x00};*/
  /*u8 bar[4] = {0x22, 0x22, 0x22, 0x22};*/
  /*swift_nap_xfer(0x00, 4, bar, foo); */
  /*printf("First:\n");*/
  /*for (int i=0; i<4; i++)*/
    /*printf("0x%02X\n", bar[i]);*/
  /*u8 n = 0;*/
  /*while(1) ;*/
  /*{*/
    /*if (foo[0] != bar[0] || foo[1] != bar[1] || foo[2] != bar[2] || foo[3] != bar[3])*/
    /*{*/
      /*printf("oops:\n");*/
      /*for (int i=0; i<4; i++)*/
        /*printf("0x%02X, 0x%02X\n", bar[i], foo[i]);*/
    /*}*/
    /*n++;*/
    /*foo[3] = n;*/
    /*swift_nap_xfer(0x00, 4, bar, foo); */
    /*[>for (u32 i = 0; i < 6000000; i++)<]*/
      /*[>__asm__("nop");<]*/
  /*}*/


  led_toggle(LED_RED);
  
  printf(" Set 'save data' mode\n");
  acq_set_load_enable();

  printf(" Timing stroble\n");
  u32 cnt = timing_count();
  timing_strobe(cnt + 1000);
  printf(" Current count %d, Strobe on %d\n", (unsigned int)cnt, (unsigned int)cnt+1000);

  printf(" Wait for data capture interrupt from FPGA\n");
  while(last_exti_count() <= cnt);
  printf(" Got exti at count %d\n", (unsigned int)last_exti_count());

  printf(" Unsetting 'save data' mode (clears IRQ)\n");
  acq_clear_load_enable();

  printf(" Writing acq parameters\n");
  u32 temp = 0;
  u32 last_last_exti = last_exti_count();
  temp = acq_init(20, 10628, -35); 
  printf("  acq reg: 0x%08X\n", (unsigned int)temp);

  printf("Wait for IRQ\n");
  while(last_exti_count() <= last_last_exti);
  printf(" Got exti at count %d\n", (unsigned int)last_exti_count());


  printf("Done!\n");

  while (1);
  
	return 0;
}
