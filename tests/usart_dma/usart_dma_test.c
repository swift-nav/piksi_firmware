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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/dma.h>

#include "error.h"
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

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  usarts_setup(USART_DEFUALT_BAUD, USART_DEFUALT_BAUD, USART_DEFUALT_BAUD);

  /*printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");*/
  /*printf("--- USART DMA TEST ---\n");*/

  #define MAX_TX ((u32)(USART_TX_BUFFER_LEN*1.2))

  u8 guard_below[30];
  u8 buff_out[MAX_TX];
  u8 guard_above[30];
  u32 len;

  for (u8 i=0; i<30; i++) {
    guard_below[i] = 0;
    guard_above[i] = 0;
  }

  for (u32 i=0; i<MAX_TX; i++)
    buff_out[i] = (u8)i;

  while(1) {
    /* Random transmit length. */
    len = (u32)rand() % MAX_TX;
    while (len)
      len -= usart_write_dma(&ftdi_tx_state, buff_out, len);

    /* Check the guards for buffer over/underrun. */
    for (u8 i=0; i<30; i++) {
      if (guard_below[i] != 0)
        screaming_death();
      if (guard_above[i] != 0)
        screaming_death();
    }

    /* Introduce some timing jitter. */
    for (u32 i = 0; i < ((u32)rand() % 10000); i++)
      __asm__("nop");
  }

  while (1);

	return 0;
}

