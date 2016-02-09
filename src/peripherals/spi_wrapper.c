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

#include <string.h>

#include <ch.h>
#include <hal.h>

#include "spi_wrapper.h"

static MUTEX_DECL(spi_mutex);

static const struct {
  SPIDriver *driver;
  SPIConfig config;
} spi_slave[] = {
  [SPI_SLAVE_FPGA] = {&SPID1, {NULL, GPIOA, GPIOA_SPI1NSS, 0}},
  [SPI_SLAVE_FLASH] = {&SPID2, {NULL, GPIOB, GPIOB_SPI2NSS_FLASH, 0}},
  [SPI_SLAVE_FRONTEND] = {&SPID2, {NULL, GPIOB, GPIOB_SPI2NSS_MAX, 0}},
};

/** Set up the SPI buses.
 * Set up the SPI peripheral, SPI clocks, SPI pins, and SPI pins' clocks.
 */
void spi_setup(void)
{

  palSetPadMode(GPIOA, GPIOA_SPI1NSS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_SPI1SCK, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOA, GPIOA_SPI1MISO, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOA, GPIOA_SPI1MOSI, PAL_MODE_ALTERNATE(5));

  palSetPadMode(GPIOB, GPIOB_SPI2NSS_MAX, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, GPIOB_SPI2NSS_FLASH, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, GPIOB_SPI2SCK, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOB, GPIOB_SPI2MISO, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOB, GPIOB_SPI2MOSI, PAL_MODE_ALTERNATE(5));
}

/** Deactivate SPI buses.
 * Disable SPI peripherals, SPI clocks, High-Z SPI pins, and disable SPI pins'
 * clocks.
 */
void spi_deactivate(void)
{
  palSetPadMode(GPIOA, GPIOA_SPI1NSS, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, GPIOA_SPI1SCK, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, GPIOA_SPI1MISO, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, GPIOA_SPI1MOSI, PAL_MODE_INPUT);

  palSetPadMode(GPIOB, GPIOB_SPI2NSS_MAX, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, GPIOB_SPI2NSS_FLASH, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, GPIOB_SPI2SCK, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, GPIOB_SPI2MISO, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, GPIOB_SPI2MOSI, PAL_MODE_INPUT);
}

/** Drive SPI nCS line low for selected peripheral.
 * \param slave Peripheral to drive chip select for.
 */
void spi_slave_select(u8 slave)
{
  chMtxLock(&spi_mutex);
  spiStart(spi_slave[slave].driver, &spi_slave[slave].config);
  spiSelect(spi_slave[slave].driver);
}

/** Drive all SPI nCS lines high.
 * Should be called after an SPI transfer is finished.
 */
void spi_slave_deselect(u8 slave)
{
  spiUnselect(spi_slave[slave].driver);
  spiStop(spi_slave[slave].driver);

  chMtxUnlock(&spi_mutex);
}

u8 spi_slave_xfer(u8 slave, u8 data)
{
  return spiPolledExchange(spi_slave[slave].driver, data);
}

void spi_slave_xfer_async(u8 slave, u16 n_bytes, u8 data_in[], const u8 data_out[])
{
  if (data_in != NULL)
    spiExchange(spi_slave[slave].driver, n_bytes, data_out, data_in);
  else
    spiSend(spi_slave[slave].driver, n_bytes, data_out);
}

/** \} */

/** \} */

