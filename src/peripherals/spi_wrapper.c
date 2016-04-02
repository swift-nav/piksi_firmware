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
#include <assert.h>

#include <ch.h>
#include <hal.h>

#include "spi_wrapper.h"

typedef struct {
  SPIDriver *driver;
  mutex_t mutex;
  thread_t *owner;
  u8 lock_nest;
  u8 active_slave;
} spi_bus_t;

typedef struct {
  spi_bus_t *bus;
  SPIConfig config;
} spi_slave_t;

static spi_bus_t spi_bus_1 = {
  &SPID1, _MUTEX_DATA(spi_bus_1.mutex), NULL, 0, SPI_SLAVE_MAX
};

static spi_bus_t spi_bus_2 = {
  &SPID2, _MUTEX_DATA(spi_bus_2.mutex), NULL, 0, SPI_SLAVE_MAX
};

static const spi_slave_t spi_slave[] = {
  [SPI_SLAVE_FPGA] = {&spi_bus_1, {NULL, PAL_PORT(LINE_SPI1NSS), PAL_PAD(LINE_SPI1NSS), 0, false}},
  [SPI_SLAVE_FLASH] = {&spi_bus_2, {NULL, PAL_PORT(LINE_SPI2NSS_FLASH), PAL_PAD(LINE_SPI2NSS_FLASH), 0, true}},
  [SPI_SLAVE_FRONTEND] = {&spi_bus_2, {NULL, PAL_PORT(LINE_SPI2NSS_MAX), PAL_PAD(LINE_SPI2NSS_MAX), 0, true}},
};

/** Set up the SPI buses.
 * Set up the SPI peripheral, SPI clocks, SPI pins, and SPI pins' clocks.
 */
void spi_setup(void)
{
  palSetLineMode(LINE_SPI1NSS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_SPI1SCK, PAL_MODE_ALTERNATE(5));
  palSetLineMode(LINE_SPI1MISO, PAL_MODE_ALTERNATE(5));
  palSetLineMode(LINE_SPI1MOSI, PAL_MODE_ALTERNATE(5));

  palSetLineMode(LINE_SPI2NSS_MAX, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_SPI2NSS_FLASH, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_SPI2SCK, PAL_MODE_ALTERNATE(5));
  palSetLineMode(LINE_SPI2MISO, PAL_MODE_ALTERNATE(5));
  palSetLineMode(LINE_SPI2MOSI, PAL_MODE_ALTERNATE(5));
}

/** Deactivate SPI buses.
 * Disable SPI peripherals, SPI clocks, High-Z SPI pins, and disable SPI pins'
 * clocks.
 */
void spi_deactivate(void)
{
  palSetLineMode(LINE_SPI1NSS, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI1SCK, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI1MISO, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI1MOSI, PAL_MODE_INPUT);

  palSetLineMode(LINE_SPI2NSS_MAX, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI2NSS_FLASH, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI2SCK, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI2MISO, PAL_MODE_INPUT);
  palSetLineMode(LINE_SPI2MOSI, PAL_MODE_INPUT);
}

/** Lock the SPI bus used by the selected peripheral.
 * \note This function may be called before spi_slave_select() to enforce
 * exclusive access to the SPI bus across multiple transactions.
 * \param slave Peripheral to lock the SPI bus for.
 */
void spi_lock(u8 slave)
{
  spi_bus_t *bus = spi_slave[slave].bus;
  thread_t *thread = chThdGetSelfX();

  chSysLock();
  if (bus->owner != thread) {
    chMtxLockS(&bus->mutex);
    bus->owner = thread;
  } else {
    bus->lock_nest++;
  }
  chSysUnlock();
}

/** Unlock the SPI bus used by the selected peripheral.
 * \note This function should be called after spi_slave_deselect() if
 * the bus was locked with spi_lock().
 * \param slave Peripheral to unlock the SPI bus for.
 */
void spi_unlock(u8 slave)
{
  spi_bus_t *bus = spi_slave[slave].bus;

  if (bus->lock_nest > 0) {
    bus->lock_nest--;
  } else {
    bus->owner = NULL;
    chMtxUnlock(&bus->mutex);
  }
}

/** Drive SPI nCS line low for selected peripheral.
 * \param slave Peripheral to drive chip select for.
 */
void spi_slave_select(u8 slave)
{
  spi_lock(slave);

  const spi_slave_t *s = &spi_slave[slave];
  spi_bus_t *bus = s->bus;
  SPIDriver *driver = bus->driver;

  if (bus->active_slave != slave) {
    spiStop(driver);
    spiStart(driver, &s->config);
    bus->active_slave = slave;
  }
  spiSelect(driver);
}

/** Drive all SPI nCS lines high.
 * Should be called after an SPI transfer is finished.
 */
void spi_slave_deselect(u8 slave)
{
  SPIDriver *driver = spi_slave[slave].bus->driver;
  spiUnselect(driver);
  spi_unlock(slave);
}

u8 spi_slave_xfer(u8 slave, u8 data)
{
  SPIDriver *driver = spi_slave[slave].bus->driver;
  return spiPolledExchange(driver, data);
}

/* Note: buffers must NOT be in CCM */
void spi_slave_xfer_dma(u8 slave, u16 n_bytes, u8 data_in[], const u8 data_out[])
{
  SPIDriver *driver = spi_slave[slave].bus->driver;
  if (data_in != NULL) {
    spiExchange(driver, n_bytes, data_out, data_in);
  } else {
    spiSend(driver, n_bytes, data_out);
  }
}

/** \} */

/** \} */

