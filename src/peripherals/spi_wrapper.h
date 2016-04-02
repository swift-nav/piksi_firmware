/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#include <hal.h>
#include <libswiftnav/common.h>

/** \addtogroup spi
 * \{ */

enum {
  SPI_SLAVE_FPGA,         /**< SwiftNAP FPGA */
  SPI_SLAVE_FLASH,        /**< M25 configuration flash */
  SPI_SLAVE_FRONTEND,     /**< MAX2769 front-end */
  SPI_SLAVE_MAX
};

#define SPI_BUS_FLASH    SPID2 /**< SPI bus that the M25 flash is on. */
#define SPI_BUS_FPGA     SPID1 /**< SPI bus that the FPGA is on. */
#define SPI_BUS_FRONTEND SPID2 /**< SPI bus that the MAX2769 is on. */

/** \} */

#define SPI_USE_ASYNC    TRUE

void spi_setup(void);
void spi_deactivate(void);
void spi_lock(u8 slave);
void spi_unlock(u8 slave);
void spi_slave_select(u8 slave);
void spi_slave_deselect(u8 slave);
u8 spi_slave_xfer(u8 slave, u8 data);
void spi_slave_xfer_dma(u8 slave, u16 n_bytes, u8 data_in[], const u8 data_out[]);

#endif

