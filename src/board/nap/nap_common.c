/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libsbp/sbp.h>

#include "../../error.h"
#include "../../peripherals/spi.h"
#include "../../sbp.h"
#include "../../init.h"
#include "../max2769.h"
#include "nap_conf.h"
#include "nap_common.h"
#include "nap_exti.h"

#include <ch.h>

BinarySemaphore timing_strobe_sem;

/** \addtogroup board
 * \{ */

/** \defgroup nap SwiftNAP
 * Interface to the Swift Navigation Acceleration Peripheral.
 * Contains functions for interacting with FPGA specific aspects of the
 * SwiftNAP (e.g. controlling when it configures itself, retrieving
 * information about the configuration, resetting the internal logic) and
 * functions for interacting with the SwiftNAP internal register interface.
 * \{ */

/** Set up peripherals and parts of receiver related to or depended on by NAP.
 * Sets up GPIOs associated with NAP, waits for NAP to finish configuring, sets
 * up SPI, sets up MAX2769 Frontend, sets up NAP interrupt, sets up NAP
 * callbacks, gets NAP configuration parameters from FPGA flash.
 */
void nap_setup()
{
  /* Setup the FPGA conf done line. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);

  /* Setup the FPGA hash read done line. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);

  /* Set up the FPGA CONF_B line. */
  nap_conf_b_setup();
  /* Force FPGA to delay configuration (i.e. use of SPI2) by setting it low. */
  nap_conf_b_clear();

  /* Initialise the SPI peripheral so that we can set up the RF frontend. */
  spi_setup();
  //spi_dma_setup();

  /* Configure the front end. */
  max2769_configure();

  /* Deactivate SPI buses so the FPGA can use the SPI2 bus to configure. */
  spi_deactivate();

  /* Allow the FPGA to configure. */
  nap_conf_b_set();

  /* Wait for FPGA to finish configuring (uses SPI2 bus). */
  while (!(nap_conf_done())) ;

  /* Wait for FPGA to read authentication hash out of flash (uses SPI2 bus). */
  while (!(nap_hash_rd_done())) ;

  /* FPGA is done using SPI2: re-initialise the SPI peripheral. */
  spi_setup();
  spi1_dma_setup();

  /* Switch the STM's clock to use the Frontend clock from the NAP */
  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);

  /* Set up the NAP interrupt line. */
  nap_exti_setup();

  /* Get NAP parameters (number of acquisition taps, number of tracking
   * channels, etc) from configuration flash.
   */
  nap_conf_rd_parameters();

  chBSemInit(&timing_strobe_sem, TRUE);
}

/** Check if NAP configuration is finished.
 *
 * \return 1 if configuration is finished (line high), 0 if not finished (line
 *low).
 */
u8 nap_conf_done(void)
{
  return gpio_get(GPIOC, GPIO1) ? 1 : 0;
}

/** Setup the GPIO for the FPGA CONF B pin. */
void nap_conf_b_setup(void)
{
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set(GPIOC, GPIO12);
}

/** Set the FPGA CONF B pin high.
 * Allows FPGA to use SPI2 lines and configure itself.
 */
void nap_conf_b_set(void)
{
  gpio_set(GPIOC, GPIO12);
}

/** Set the FPGA CONF B pin low.
 * Delays FPGA from using SPI2 lines and configuring itself as long as CONF B is
 *low.
 */
void nap_conf_b_clear(void)
{
  gpio_clear(GPIOC, GPIO12);
}

/** See if NAP has finished reading authentication hash from configuration
 * flash.
 *
 * \return 1 if hash read has finished else 0
 */
u8 nap_hash_rd_done(void)
{
  return gpio_get(GPIOA, GPIO3) ? 0 : 1;
}

/** Return status of NAP authentication hash comparison.
 *
 * \return Status of NAP authentication hash comparison.
 */
u8 nap_hash_status(void)
{
  u8 temp[1] = { 0 };

  nap_xfer_blocking(NAP_REG_HASH_STATUS, 1, temp, temp);
  return temp[0];
}

/** Get Device DNA from Spartan 6 FPGA.
 *
 * \param dna Array to insert DNA into (length 8).
 */
void nap_rd_dna(u8 dna[])
{
  nap_xfer_blocking(NAP_REG_DNA, 8, dna, dna);
}

/** Send Device DNA from Spartan 6 FPGA over Piksi binary USARTs.
 *
 * \param buff Unused argument, callback takes no input.
 */
void nap_rd_dna_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;
  u8 dna[8];
  nap_rd_dna(dna);
  sbp_send_msg(MSG_NAP_DEVICE_DNA, 8, dna);
}

/** Setup NAP callbacks. */
void nap_callbacks_setup(void)
{
  static sbp_msg_callbacks_node_t nap_dna_node;

  sbp_register_cbk(MSG_NAP_DEVICE_DNA, &nap_rd_dna_callback,
                   &nap_dna_node);
}

/** Do an SPI transfer to/from one of the NAP's internal registers.
 *
 * \param reg_id   NAP register ID.
 * \param n_bytes  Number of bytes to transfer to/from register.
 * \param data_in  Array of length n_bytes to transfer to NAP register.
 * \param data_out Array of length n_bytes to transfer NAP register data into.
 */
void nap_xfer_blocking(u8 reg_id, u16 n_bytes, u8 data_in[],
                       const u8 data_out[])
{
  spi_slave_select(SPI_SLAVE_FPGA);

  spi_xfer(SPI_BUS_FPGA, reg_id);

  /* Spin for shorter transfers to avoid the overhead of context switching. */
  if (n_bytes < 8) {
    /* If data_in is NULL then discard read data. */
    if (data_in)
      for (u16 i = 0; i < n_bytes; i++)
        data_in[i] = spi_xfer(SPI_BUS_FPGA, data_out[i]);
    else
      for (u16 i = 0; i < n_bytes; i++)
        spi_xfer(SPI_BUS_FPGA, data_out[i]);
  } else {
        spi1_xfer_dma(n_bytes, data_in, data_out);
  }
  spi_slave_deselect();
}

/** Get the current NAP internal sample clock count.
 * NAP's internal count of sample clocks + (number of NAP's counter rollovers)
 * times 2^32. NAP's internal sample clock counter is 32 bits wide - at a
 * 16.368MHz sample clock frequency it rolls over approximately every 262
 * seconds.
 *
 * \return NAP's internal count of sample clocks +
 *               (total number of NAP counter rollovers) * 2^32.
 */
u64 nap_timing_count(void)
{
  static u32 rollover_count = 0;
  static u32 prev_count = 0;

  u8 temp[4] = { 0, 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_TIMING_COUNT, 4, temp, temp);

  u32 count = (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];

  if (count < prev_count)
    rollover_count++;

  prev_count = count;

  return (u64)count | ((u64)rollover_count << 32);
}

/** Get the count of NAP's internal sample clock counter at the clock cycle the
 * last timing_strobe went high.
 *
 * Allows checking that there is enough timing margin between the count written
 * with timing_strobe and the count of the NAP's internal timer at the clock
 * cycle the timing_strobe count is latched in.
 *
 * \return Count of NAP's internal timer (latched at the cycle the last
 *         timing_strobe went high)
 */
u32 nap_timing_count_latched(void)
{
  u8 temp[4] = { 0, 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_TIMING_COUNT_LATCH, 4, temp, temp);
  return (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];
}

/** Schedule a NAP timing strobe.
 * Sets NAP's internal timing strobe high. It will go low when NAP's internal
 * sample clock counter reaches falling_edge_count. Various events in the NAP
 * are triggered by the falling edge of the timing strobe, e.g. loading the
 * acquisition channel sample ram or starting tracking channels at a known
 * time.
 *
 * \param falling_edge_count The value of the NAP's internal counter at which
 *                           the internal timing strobe is to be cleared.
 */
void nap_timing_strobe(u32 falling_edge_count)
{
  u8 temp[4];

  temp[0] = (falling_edge_count >> 24) & 0xFF;
  temp[1] = (falling_edge_count >> 16) & 0xFF;
  temp[2] = (falling_edge_count >> 8) & 0xFF;
  temp[3] = (falling_edge_count >> 0) & 0xFF;
  nap_xfer_blocking(NAP_REG_TIMING_COMPARE, 4, temp, temp);

  /* TODO: Check the value read from the latched count register to check that
   * the timing strobe value requested really was in the future. */

  /* TODO: need to wait until the timing strobe has finished but also don't
   * want to spin in a busy loop. */

  /* Add a little bit of delay before the next
   * timing strobe.
   */
  for (u32 i = 0; i < 50; i++)
    __asm__("nop");
}

bool nap_timing_strobe_wait(u32 timeout)
{
  return chBSemWaitTimeout(&timing_strobe_sem, timeout) == RDY_RESET;
}

/** Read NAP's error register.
 * NAP's error register shows if any channel has had a pipelining error (a
 * second IRQ occurs before the first has been serviced).
 *
 * \return 32 bit value from NAP's error register.
 */
u32 nap_error_rd_blocking(void)
{
  u8 temp[4] = { 0, 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_ERROR, 4, temp, temp);
  return (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];
}

/** \} */

/** \} */

