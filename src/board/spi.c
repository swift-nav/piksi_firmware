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

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/f2/rcc.h>

#include "spi.h"

void spi_setup(void)
{
  /* Enable SPI periperal clock */
  RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
  /* Enable GPIO clocks for CS lines */
	RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  /* Setup CS line GPIOs */
  spi_slave_deselect();
  /* FPGA CS */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO8);
  /* Configuration flash CS */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO12);
  /* Front-end CS */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);

  /* Setup SPI alternate function */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
  /*gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO13 | GPIO15);*/
	gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);

	/* Setup SPI parameters. */
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_16, 0, \
      0, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_ss_output(SPI2); /* Required, see 25.3.1 section about NSS */

	/* Finally enable the SPI. */
	spi_enable(SPI2);
}

void spi_slave_select(u8 slave)
{
  u32 i;

  spi_slave_deselect();

  for (i = 0; i < 6; i++)
    __asm__("nop");

  switch (slave) {
    case SPI_SLAVE_FPGA:
      gpio_clear(GPIOC, GPIO8);
      break;
    case SPI_SLAVE_FLASH:
      gpio_clear(GPIOB, GPIO12);
      break;
    case SPI_SLAVE_FRONTEND:
      gpio_clear(GPIOB, GPIO11);
      break;
  }

  for (i = 0; i < 6; i++)
    __asm__("nop");
}

void spi_slave_deselect(void)
{
  /* Deselect FPGA CS */
  gpio_set(GPIOC, GPIO8);
  /* Deselect Configuration flash and Front-end CS */
  gpio_set(GPIOB, GPIO11 | GPIO12);
}

