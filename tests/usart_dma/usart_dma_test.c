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
#include <string.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "hw/leds.h"
#include "hw/usart.h"

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

void death() {
  while(1);
}

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  usart_setup_common();
  usart_tx_dma_setup();

  /*printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");*/
  /*printf("--- USART DMA TEST ---\n");*/

  u8 guard_below[30];
  u8 buff_out[300];
  u8 guard_above[30];
  u8 len;

  for (int i=0; i<30; i++) {
    guard_below[i] = 0;
    guard_above[i] = 0;
  }

  for (int i=0; i<255; i++)
    buff_out[i] = i;

  while(1)
  {

    len = 255;
    while (len)
      len -= usart_write_dma(buff_out, len);
    /*len = 0;*/
    /*while (len < 255) {*/
      /*while(usart_n_read_dma() == 0);*/
      /*len += usart_read_dma(zee_buff_in, 255);*/
    /*}*/
    /*if (memcmp(zee_buff_in, zee_buff_out, 255) != 0) {*/
      /*death();*/
    /*}*/
  /*for (u32 i = 0; i < 600000; i++)*/
    /*__asm__("nop");*/


    /* Check the guards for buffer over/underrun. */
    for (int i=0; i<30; i++) {
      if (guard_below[i] != 0)
        death();
      if (guard_above[i] != 0)
        death();
    }
  }

  while (1);

	return 0;
}

