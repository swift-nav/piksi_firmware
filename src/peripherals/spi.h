/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SPI_H
#define SWIFTNAV_SPI_H

#include <libswiftnav/common.h>
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

