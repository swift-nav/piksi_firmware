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

#include "swift_nap.h"
#include "board/spi.h"

u32 swift_nap_xfer(u8 spi_id, u8 addr, u32 data)
{
  u32 data_in = 0;

  spi_slave_select(SPI_SLAVE_FPGA);

  spi_xfer(SPI_BUS_FPGA, spi_id);
  spi_xfer(SPI_BUS_FPGA, addr);

  data_in |= (u32)spi_xfer(SPI_BUS_FPGA, (data >>  0) & 0xFF) <<  0;
  data_in |= (u32)spi_xfer(SPI_BUS_FPGA, (data >>  8) & 0xFF) <<  8;
  data_in |= (u32)spi_xfer(SPI_BUS_FPGA, (data >> 16) & 0xFF) << 16;
  data_in |= (u32)spi_xfer(SPI_BUS_FPGA, (data >> 24) & 0xFF) << 24;

  spi_slave_deselect();

  return data_in;
}

u32 swift_nap_read(u8 spi_id, u8 addr)
{
  return swift_nap_xfer(spi_id, addr & 0x7F, 0);
}

void swift_nap_write(u8 spi_id, u8 addr, u32 data)
{
  swift_nap_xfer(spi_id, addr | 0x80, data);
}
