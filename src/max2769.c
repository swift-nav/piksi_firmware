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

#include "board/spi.h"

#include "max2769.h"

void max2769_write(u8 addr, u32 data)
{
  u32 write_word = ((data << 4) & 0xFFFFFFF0) | (addr & 0x0F);

  spi_slave_select(SPI_SLAVE_FRONTEND);

  spi_xfer(SPI_BUS_FRONTEND, (write_word >> 24) & 0xFF);
  spi_xfer(SPI_BUS_FRONTEND, (write_word >> 16) & 0xFF);
  spi_xfer(SPI_BUS_FRONTEND, (write_word >>  8) & 0xFF);
  spi_xfer(SPI_BUS_FRONTEND, (write_word >>  0) & 0xFF);

  spi_slave_deselect();

}

void max2769_setup()
{
  /* Register settings from Colin "max_regs_defaults.vhd" */
  max2769_write(MAX2769_CONF1, 0xA2939A3);
  max2769_write(MAX2769_CONF2, 0x8550308);
  max2769_write(MAX2769_CONF3, 0xEAFE1DC);
  max2769_write(MAX2769_PLLCONF, 0x9EC0008);
  max2769_write(MAX2769_DIV, 0x0C00080);
  max2769_write(MAX2769_FDIV, 0x8000070);
  max2769_write(MAX2769_STRM, 0x8000000);
  max2769_write(MAX2769_CLK, 0x10061B2);
}
