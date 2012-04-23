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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "debug.h"
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

msg_callbacks_node_t foo_callback_node;
void foo_callback(u8 buff[])
{
  printf("Foo callback: %f\n", *((float*)buff));
}

msg_callbacks_node_t led_callback_node;
void led_callback(u8 buff[])
{
  printf("Green LED: ");
  if (buff[0] & 1) {
    led_on(LED_GREEN);
    printf("ON");
  } else {
    led_off(LED_GREEN);
    printf("OFF");
  }

  printf(", Red LED: ");
  if (buff[0] & 2) {
    led_on(LED_RED);
    printf("ON\n");
  } else {
    led_off(LED_RED);
    printf("OFF\n");
  }
}

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  debug_setup();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- DEBUG TEST ---\n");

  debug_register_callback(0x22, &foo_callback, &foo_callback_node);
  debug_register_callback(0x42, &led_callback, &led_callback_node);
  while(1)
  {
    debug_process_messages();
    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }

  while (1);

	return 0;
}

