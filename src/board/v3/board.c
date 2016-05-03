/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdlib.h>
#include "hal.h"
#include "zynq7000.h"

#define LED_GPIO_LINE PAL_LINE(GPIO1, 15)
#define BUTTON_GPIO_LINE PAL_LINE(GPIO1, 19)

#define SPI_MOSI_GPIO_LINE PAL_LINE(GPIO0, 10)
#define SPI_MISO_GPIO_LINE PAL_LINE(GPIO0, 11)
#define SPI_CLK_GPIO_LINE PAL_LINE(GPIO0, 12)
#define SPI_SS_GPIO_LINE PAL_LINE(GPIO0, 13)

const PALConfig pal_default_config;
const WDGConfig board_wdg_config = {
  .period_ms = 30000,
};

static void cycle_counter_init(void)
{
  /* Set up TTC0_2 with period of ZYNQ7000_CPU_1x_FREQUENCY_Hz / 2^10 */
  TTC0->CLKCTRL[2] =  (TTC_CLKCTRL_SRC_PCLK << TTC_CLKCTRL_SRC_Pos) |
                      (9 << TTC_CLKCTRL_PSVAL_Pos) |
                      (1 << TTC_CLKCTRL_PSEN_Pos);
  TTC0->INTERVAL[2] = 0xffff;
  TTC0->CNTCTRL[2] =  (1 << TTC_CNTCTRL_RESET_Pos) |
                      (1 << TTC_CNTCTRL_INTERVAL_Pos);
}

/*
 * Board-specific initialization code.
 */
void boardInit(void)
{
  /* Unlock SLCR */
  *(volatile uint32_t *)0xF8000008 = 0xDF0D;

  /* Enable UART0 and UART1 clocks */
  *(volatile uint32_t *)0xF800012C |= (1 << 20) | (1 << 21);

  /* UART REFCLK = 1GHz / 20 = 50MHz */
  *(volatile uint32_t *)0xF8000154 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000154 |= (20 << 8);
  *(volatile uint32_t *)0xF8000154 |= (1 << 0) | (1 << 1);

  /* Enable SPI0 and SPI1 clocks */
  *(volatile uint32_t *)0xF800012C |= (1 << 14) | (1 << 15);

  /* SPI REFCLK = 1GHz / 20 = 50MHz */
  *(volatile uint32_t *)0xF8000158 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000158 |= (20 << 8);
  *(volatile uint32_t *)0xF8000158 |= (1 << 0) | (1 << 1);

  /* Assert FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0xf;

  /* FPGA_CLK0 = 1GHz / 10 = 100MHz */
  *(volatile uint32_t *)0xF8000170 &= ~(0x3F << 20);
  *(volatile uint32_t *)0xF8000170 |= (1 << 20);
  *(volatile uint32_t *)0xF8000170 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000170 |= (10 << 8);

  /* Release FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0x0;

  /* Configure button and LED pins */
  palSetLineMode(BUTTON_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(LED_GPIO_LINE, PAL_MODE_OUTPUT_PUSHPULL);

  /* Configure frontend SPI pins */
  palSetLineMode(SPI_MOSI_GPIO_LINE, PAL_MODE_CUSTOM(PAL_DIR_PERICTRL, PAL_PULL_NONE, PAL_SPEED_SLOW, PAL_PIN_FUNCTION(5 << 4)));
  palSetLineMode(SPI_MISO_GPIO_LINE, PAL_MODE_CUSTOM(PAL_DIR_INPUT, PAL_PULL_NONE, PAL_SPEED_SLOW, PAL_PIN_FUNCTION(5 << 4)));
  palSetLineMode(SPI_CLK_GPIO_LINE, PAL_MODE_CUSTOM(PAL_DIR_PERICTRL, PAL_PULL_NONE, PAL_SPEED_SLOW, PAL_PIN_FUNCTION(5 << 4)));
  palSetLineMode(SPI_SS_GPIO_LINE, PAL_MODE_OUTPUT_PUSHPULL);

  cycle_counter_init();
}

void board_preinit_hook(void)
{
}

