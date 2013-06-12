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

#ifndef SWIFTNAV_SPI_H
#define SWIFTNAV_SPI_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/spi.h>

/** \addtogroup spi
 * \{ */

#define SPI_SLAVE_FPGA     0x01
#define SPI_SLAVE_FLASH    0x02
#define SPI_SLAVE_FRONTEND 0x03

#define SPI_BUS_FLASH    SPI2
#define SPI_BUS_FPGA     SPI1
#define SPI_BUS_FRONTEND SPI2

/** \} */

void spi_setup(void);
void spi_deactivate(void);
void spi_slave_select(u8 slave);
void spi_slave_deselect(void);

#endif

