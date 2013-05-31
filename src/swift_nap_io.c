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

#include <math.h>
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/timer.h>

#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "cw.h"
#include "debug.h"
#include "error.h"
#include "hw/spi.h"
#include "hw/max2769.h"
#include "hw/m25_flash.h"

#include <libswiftnav/prns.h>

#define FLASH_NAP_PARAMS_ADDR 0xD0000
#define FLASH_NAP_GIT_HASH_ADDR 0xE0000
#define FLASH_NAP_GIT_UNCLEAN_ADDR (FLASH_NAP_GIT_HASH_ADDR + 20)

/* NAP Parameters stored in the FPGA configuration flash. */
u8 ACQ_N_TAPS;
u8 TRACK_N_CHANNELS;

u32 exti_count = 0;

/* SPI DMA is not implemented yet. */
#define SPI_DMA_BUFFER_LEN 22
u8 spi_dma_buffer[SPI_DMA_BUFFER_LEN];

/** Get Device DNA from Spartan 6 FPGA.
 *
 * \param dna Array to insert DNA into (length 8).
 */
void get_nap_dna(u8 dna[])
{
  swift_nap_xfer_blocking(SPI_ID_DNA,8,dna,dna);
}

/** Send Device DNA from Spartan 6 FPGA over Piksi binary USARTs. */
void get_nap_dna_callback()
{
  u8 dna[8];
  get_nap_dna(dna);
  debug_send_msg(MSG_NAP_DEVICE_DNA, 8, dna);
}

/** Setup NAP callbacks. */
void swift_nap_callbacks_setup()
{
  static msg_callbacks_node_t swift_nap_dna_node;
  debug_register_callback(MSG_NAP_DEVICE_DNA, &get_nap_dna_callback, &swift_nap_dna_node);
}

/** Get NAP configuration parameters from FPGA configuration flash.
 * Gets information about the NAP configuration (number of code phase taps in
 * the acquisition channel, number of tracking channels, etc).
 */
void get_nap_parameters()
{
  /* Define parameters that need to be read from FPGA configuration flash.
   * Pointers in the array should be in the same order they're stored in the
   * configuration flash. */
  u8 * nap_parameters[2] = {
                            &ACQ_N_TAPS,
                            &TRACK_N_CHANNELS
                           };
  /* Get parameters from FPGA configuration flash */
  for (u8 i=0; i<(sizeof(nap_parameters)/sizeof(nap_parameters[0])); i++){
    m25_read(FLASH_NAP_PARAMS_ADDR + i, 1, nap_parameters[i]);
  }
  /* Other parameters that are derived from NAP parameters */
}

/** Set up peripherals and parts of receiver related to or depended on by NAP.
 * Sets up GPIOs associated with NAP, waits for NAP to finish configuring, sets
 * up SPI, sets up MAX2769 Frontend, sets up NAP interrupt, sets up NAP
 * callbacks, gets NAP configuration parameters from FPGA flash.
 */
void swift_nap_setup()
{
  /* Setup the FPGA conf done line */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);

  /* Setup the FPGA hash read done line */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);

  /* We don't want spi_setup() called until the FPGA has finished configuring
   * itself and has read the device hash out of the configuration flash.
   * (It uses the SPI2 bus for this.)
   */
  /* TODO: Timeout here if FPGA doesn't configure in expected time? */
  while (!(swift_nap_conf_done() && swift_nap_hash_rd_done()));

  /* Initialise the SPI peripheral. */
  spi_setup();
  //spi_dma_setup();

  /* Setup the front end. */
  max2769_setup();

  /* Setup the reset line GPIO */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
  gpio_clear(GPIOA, GPIO2);

  /* Setup the external interrupts. */
  exti_setup();

  /* Setup callback functions */
  swift_nap_callbacks_setup();

  /* Get NAP parameters (number of acquisition taps, number of tracking
   * channels, etc) from flash
   */
  get_nap_parameters();
}

/** Reset logic inside NAP. */
void swift_nap_reset()
{
  gpio_set(GPIOA, GPIO2);
  for (int i = 0; i < 50; i++)
    __asm__("nop");
  gpio_clear(GPIOA, GPIO2);
  for (int i = 0; i < 200; i++)
    __asm__("nop");
}

/** Check if FPGA configuration is finished.
 *
 * \return 1 if configuration is finished (line high), 0 if not finished (line low).
 */
u8 swift_nap_conf_done()
{
  return gpio_get(GPIOC,GPIO1) ? 1 : 0;
}

/** Setup the GPIO for the FPGA CONF B pin */
void swift_nap_conf_b_setup()
{
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set(GPIOC, GPIO12);
}

/** Set the FPGA CONF B pin high.
 * Allows FPGA to use SPI2 lines and configure itself.
 */
void swift_nap_conf_b_set()
{
  gpio_set(GPIOC, GPIO12);
}

/** Set the FPGA CONF B pin low.
 * Delays FPGA from using SPI2 lines and configuring as long as CONF B is low.
 */
void swift_nap_conf_b_clear()
{
  gpio_clear(GPIOC, GPIO12);
}

/** See if NAP has finished reading authentication hash from configuration flash.
 *
 * \return 1 if hash read has finished else 0
 */
u8 swift_nap_hash_rd_done()
{
  return gpio_get(GPIOA,GPIO3) ? 0 : 1;
}

/** Do an SPI transfer to/from one of the NAP's internal registers.
 *
 * \param spi_id   NAP register ID.
 * \param n_bytes  Number of bytes to transfer to/from register.
 * \param data_in  Array of length n_bytes to transfer to NAP register.
 * \param data_out Array of length n_bytes to transfer NAP register data into.
 */
void swift_nap_xfer_blocking(u8 spi_id, u16 n_bytes, u8 data_in[], const u8 data_out[])
{
  spi_slave_select(SPI_SLAVE_FPGA);

  spi_xfer(SPI_BUS_FPGA, spi_id);

  /* If data_in is NULL then discard read data. */
  if (data_in)
    for (u16 i=0; i < n_bytes; i++)
      data_in[i] = spi_xfer(SPI_BUS_FPGA, data_out[i]);
  else
    for (u16 i=0; i < n_bytes; i++)
      spi_xfer(SPI_BUS_FPGA, data_out[i]);

  spi_slave_deselect();
}

/** Set up NAP GPIO interrupt.
 * Interrupt is used to alert STM that a channel in the NAP needs to be serviced.
 */
void exti_setup()
{
  /* Signal from the FPGA is on PA1. */

  /* Enable clock to GPIOA. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
  /* Enable clock to SYSCFG which contains the EXTI functionality. */
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
  exti_reset_request(EXTI1);
	exti_enable_request(EXTI1);

	/* Enable EXTI1 interrupt */
	nvic_enable_irq(NVIC_EXTI1_IRQ);
}

/** NAP interrupt service routine.
 * Reads the IRQ register from NAP to determine what inside the NAP needs to be
 * serviced, and then calls the appropriate service routine.
 */
void exti1_isr()
{

  exti_reset_request(EXTI1);

  u32 irq = swift_nap_read_irq_blocking();

  if (irq & IRQ_ACQ_DONE) {
    acq_service_irq();
  }

  if (irq & IRQ_ACQ_LOAD_DONE) {
    acq_service_load_done();
  }

  if (irq & IRQ_CW_DONE) {
    cw_service_irq();
  }

  if (irq & IRQ_CW_LOAD_DONE) {
    cw_service_load_done();
  }

  /* Mask off everything but tracking irqs. */
  irq &= IRQ_TRACK_MASK;

  /* Loop over tracking irq bit flags. */
  for(u8 n=0; n<TRACK_N_CHANNELS; n++) {
    /* Save a bit of time by seeing if the rest of the bits
     * are zero in one go so we don't have to loop over all
     * of them.
     */
    if (!(irq >> n))
      break;

    /* Test if the nth tracking irq flag is set, if so service it. */
    if ((irq >> n) & 1) {
      tracking_channel_get_corrs(n);
      tracking_channel_update(n);
    }
  }

  exti_count++;

  /* We need a level (not edge) sensitive interrupt -
   * if there is another interrupt pending on the Swift
   * NAP then the IRQ line will stay high. Therefore if
   * the line is still high, trigger another interrupt.
   */
  if (GPIOA_IDR & GPIO1)
    EXTI_SWIER = (1<<1);
}

/** Get number of NAP ISR's that have occurred.
 *
 * \return Latest NAP ISR count.
 */
u32 last_exti_count()
{
  return exti_count;
}

/** Wait until next NAP ISR has occurred. */
void wait_for_exti()
{
  u32 last_last_exti = last_exti_count();
  while(last_exti_count() == last_last_exti);
}

/** Set NAP's internal timing strobe and schedule it to clear at a given time.
 *
 * \param falling_edge_count The value of the NAP's internal timer at which the
 *                           internal timing strobe is to be cleared.
 */
void timing_strobe(u32 falling_edge_count)
{
  u8 temp[4];
  temp[0] = (falling_edge_count >> 24) & 0xFF;
  temp[1] = (falling_edge_count >> 16) & 0xFF;
  temp[2] = (falling_edge_count >> 8) & 0xFF;
  temp[3] = (falling_edge_count >> 0) & 0xFF;
  swift_nap_xfer_blocking(SPI_ID_TIMING_COMPARE,4,temp,temp);

  /* TODO: need to wait until the timing strobe has finished but also don't
   * want to spin in a busy loop. */
  while(timing_count() < falling_edge_count);

  /* Add a little bit of delay before the next
   * timing strobe.
   */
  for (u32 i = 0; i < 50; i++)
    __asm__("nop");
}

/** Get the current count of NAP's internal timer.
 *
 * \return Count of NAP's internal timer (latched at beginning of register transfer)
 */
u32 timing_count(){
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_TIMING_COUNT,4,temp,temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

/** Get the count of NAP's internal timer at the clock cycle the last timing_strobe went high.
 * Allows checking that there is enough timing margin between the value written
 * with timing_strobe and the value of the NAP's internal timer at the clock
 * cycle the timing_strobe value is latched in.
 *
 * \return Count of NAP's internal timer (latched at the cycle the last timing_strobe went high)
 */
u32 timing_count_latched(){
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_TIMING_COUNT_LATCH,4,temp,temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

/** Read NAP's IRQ register.
 * NAP's IRQ register shows which channels in NAP need to be serviced.
 *
 * \return 32 bit value from NAP's IRQ register.
 */
u32 swift_nap_read_irq_blocking()
{
  u8 temp[4] = {0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_IRQ, 4, temp, temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

/** Read NAP's error register.
 * NAP's error register shows if any channel has had a pipelining error (a
 * second IRQ occurs before the first has been serviced).
 *
 * \return 32 bit value from NAP's error register.
 */
u32 swift_nap_read_error_blocking()
{
  u8 temp[4] = {0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_ERROR, 4, temp, temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

/** Set the LOAD ENABLE bit of the NAP acquisition channel's LOAD register.
 * When the LOAD ENABLE bit is set, the acquisition channel will start loading
 * samples into its sample ram, starting at the first clock cycle after the
 * NAP's internal timing strobe goes low.
 */
void acq_set_load_enable_blocking()
{
  u8 temp[1] = {0xFF};
  swift_nap_xfer_blocking(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp);
}

/** Clear the LOAD ENABLE bit of the NAP acquisition channel's LOAD register.
 * After a load to the acquisition channel's sample ram, the LOAD ENABLE bit
 * must be cleared, or future timing strobes will cause the ram to be re-loaded.
 */
void acq_clear_load_enable_blocking()
{
  u8 temp[1] = {0x00};
  swift_nap_xfer_blocking(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp);
}

/** Pack data for writing to NAP acquisition channel INIT register.
 *
 * NOTE: Swift NAP returns corrs corresponding to code phases from
 * code_phase_reg_value-ACQ_N_TAPS-1 to code_phase_reg_value where
 * code_phase_reg_value is the raw value written into the code phase
 * portion of the init register.
 *
 * <ul>
 *   <li> corrs[0] -> code_phase_reg_value-ACQ_N_TAPS+1
 *   <li> corrs[AQC_N_TAPS-1] -> code_phase_reg_value
 * </ul>
 *
 * Lets take account of this here by writing code_phase+ACQ_N_TAPS-1
 * to the code phase register on the Swift NAP. This means the
 * correlations returned will be:
 *
 * <ul>
 *   <li> corrs[0] -> code_phase
 *   <li> corrs[ACQ_N_TAPS] -> code_phase-ACQ_N_TAPS+1
 * </ul>
 *
 * \param prn          PRN number - (0..31) (deprecated)
 * \param code_phase   Code phase of the first correlation returned
 *                     (see note above), in acquisition units.
 * \param carrier_freq Carrier frequency i.e. Doppler in acquisition
 *                     units.
 */
void acq_pack_init(u8 pack[], u8 prn, u16 code_phase, s16 carrier_freq)
{
  /* Modulo 1023*4 in case adding ACQ_N_TAPS-1 rolls us over a
   * code phase boundary.
   */
  u16 code_phase_reg_value = (code_phase+ACQ_N_TAPS-1) % (1023*4);

  pack[0] = (1<<5) |                      /* Acq enabled */
            ((carrier_freq >> 7) & 0x1F); /* Carrier freq [11:7] */

  pack[1] = (carrier_freq << 1) |         /* Carrier freq [6:0] */
            (code_phase_reg_value >> 11); /* Code phase [11] */

  pack[2] = code_phase_reg_value >> 3;    /* Code phase [10:3] */

  pack[3] = (code_phase_reg_value << 5) | /* Code phase [2:0] */
            (prn & 0x1F);                 /* PRN number (0..31) */
}

/** Write acquisition parameters to NAP acquisition channel's INIT register.
 * If the channel is currently disabled, it will be enabled and start an
 * acquisition with these parameters. If it is currently enabled, another
 * acquisition will start with these parameters when the current acquisition
 * finishes.
 * NOTE: If searching more than one acquisition point for a particular PRN,
 * the second set of acquisition parameters should be written into the channel
 * as soon as possible after the first set, as they are pipelined and used
 * immediately after the first acquisition finishes. If only searching one
 * point, acq_disable_blocking should be called as soon as possible after the
 * first set of acquisition parameters are written, and again after the
 * ACQ_DONE interrupt occurs to clear the interrupt.
 *
 * \param PRN          C/A PRN to use for the acquisition (deprecated)
 * \param code_phase   Code phase of the first correlation returned
 * \param carrier_freq Carrier frequency i.e. Doppler in acquisition units.
 */
/* TODO : remove writing of PRN number to init register */
void acq_write_init_blocking(u8 prn, u16 code_phase, s16 carrier_freq)
{
  u8 temp[4];
  acq_pack_init(temp, prn, code_phase, carrier_freq);
  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, temp);
}

/** Disable NAP acquisition channel.
 * Write to the acquisition channel's INIT register to disable the correlations
 * (clears enable bit). This must be written once to pipeline disable the
 * correlations, and then a second time to clear the ACQ_DONE IRQ after the
 * last correlation has finished.
 */
void acq_disable_blocking()
{
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, temp);
}

/** Unpack correlations read from acquisition channel.
 *
 * \param packed Array of u8 data read from NAP acq channel CORR register.
 * \param corrs  Array of corr_t's of length ACQ_N_TAPS.
 */
void acq_unpack_corr(u8 packed[], corr_t corrs[])
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */
  struct {s32 xtend:24;} sign;

  for (u8 i=0; i<ACQ_N_TAPS; i++) {

    sign.xtend  = (packed[6*i]   << 16)    /* MSB */
                | (packed[6*i+1] << 8)     /* Middle byte */
                | (packed[6*i+2]);         /* LSB */

    corrs[i].Q = sign.xtend; /* Sign extend! */

    sign.xtend  = (packed[6*i+3] << 16)    /* MSB */
                | (packed[6*i+4] << 8)     /* Middle byte */
                | (packed[6*i+5]);         /* LSB */

    corrs[i].I = sign.xtend; /* Sign extend! */
  }
}

/** Read correlations from acquisition channel.
 * Must be called after the NAP IRQ register IRQ_ACQ_DONE bit goes high, before
 * the next acquisition cycle is complete.
 *
 * \param corrs Array of corr_t of length ACQ_N_TAPS
 */
void acq_read_corr_blocking(corr_t corrs[])
{
  u8 temp[2*ACQ_N_TAPS * 3];
  swift_nap_xfer_blocking(SPI_ID_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);
  acq_unpack_corr(temp, corrs);
}

/** Write CA code to acquisition channel's code ram.
 * CA Code for SV to be searched for must be written into channel's code ram
 * before acquisitions are started.
 *
 * \param prn PRN number (0-31) of CA code to be written.
 */
void acq_write_code_blocking(u8 prn)
{
  swift_nap_xfer_blocking(SPI_ID_ACQ_CODE, 128, 0, ca_code(prn));
}

/** Pack data for writing to a NAP tracking channel's INIT register.
 *
 * \param pack          Array of u8 to pack data into.
 * \param prn           CA code PRN number (0-31) to track. (deprecated)
 * \param carrier_phase Initial carrier phase.
 * \param code_phase    Initial code phase.
 */
void track_pack_init(u8 pack[], u8 prn, s32 carrier_phase, u16 code_phase)
{
  pack[0] = ((code_phase << 5) >> 16) & 0x07;
  pack[1] = (code_phase << 5) >> 8;
  pack[2] = (((carrier_phase << 5) >> 24) & 0x1F) | (code_phase << 5);
  pack[3] = (carrier_phase << 5) >> 16;
  pack[4] = (carrier_phase << 5) >> 8;
  pack[5] = (prn & 0x1F) | (carrier_phase << 5 & 0xE0);
}

/** Write to a NAP tracking channel's INIT register.
 * Sets PRN (deprecated), initial carrier phase, and initial code phase of a
 * NAP tracking channel. The tracking channel will start correlating with these
 * parameters at the falling edge of the next NAP internal timing strobe.
 * NOTE: The tracking channel's UPDATE register, which sets the carrier and
 * code phase rates, must also be written to before the internal timing strobe
 * goes low.
 *
 * \param channel       NAP tracking channel whose INIT register to write.
 * \param prn           CA code PRN (0-31) to track. (deprecated)
 * \param carrier_phase Initial code phase.
 * \param code_phase    Initial carrier phase.
 */
void track_write_init_blocking(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase)
{
  u8 temp[6] = {0, 0, 0, 0, 0, 0};
  track_pack_init(temp, prn, carrier_phase, code_phase);
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_INIT_OFFSET, 6, 0, temp);
}

/** Pack data for writing to a NAP tracking channel's UPDATE register.
 *
 * \param pack            Array of u8 to pack data into.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 */
void track_pack_update(u8 pack[], s32 carrier_freq, u32 code_phase_rate)
{
  pack[0] = (code_phase_rate >> 24) & 0x1F;
  pack[1] = (code_phase_rate >> 16);
  pack[2] = (code_phase_rate >> 8);
  pack[3] = code_phase_rate;
  pack[4] = (carrier_freq >> 8);
  pack[5] = carrier_freq;
}

/** Write to a NAP tracking channel's UPDATE register.
 * Write new carrier frequency and code phase rate to a NAP tracking channel's
 * UPDATE register, which will be used to accumulate the channel's carrier and
 * code phases during the next correlation period.
 * NOTE: This must be called in addition to track_write_init_blocking when a
 * new tracking channel is being set up, before the NAP's internal timing
 * strobe goes low.
 * NOTE: If two tracking channel IRQ's occur without a write to the tracking
 * channel's UPDATE register between them, the error bit for the tracking
 * channel in the NAP error register will go high.
 *
 * \param channel         NAP tracking channel whose UPDATE register to write.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 */
void track_write_update_blocking(u8 channel, s32 carrier_freq, u32 code_phase_rate)
{
  u8 temp[6] = {0, 0, 0, 0, 0, 0};
  track_pack_update(temp, carrier_freq, code_phase_rate);
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_UPDATE_OFFSET, 6, 0, temp);
}

/** Unpack data read from a NAP tracking channel's CORR register.
 *
 * \param packed       Array of u8 data read from channnel's CORR register.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void track_unpack_corr(u8 packed[], u16* sample_count, corr_t corrs[])
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */
  struct {s32 xtend:24;} sign;

  *sample_count = (packed[0]<<8) | packed[1];

  for (u8 i=0; i<3; i++) {

    sign.xtend  = (packed[6*(3-i-1)+2] << 16)    /* MSB */
                | (packed[6*(3-i-1)+3] << 8)     /* Middle byte */
                | (packed[6*(3-i-1)+4]);         /* LSB */

    corrs[i].Q = sign.xtend; /* Sign extend! */

    sign.xtend  = (packed[6*(3-i-1)+5] << 16)    /* MSB */
                | (packed[6*(3-i-1)+6] << 8)     /* Middle byte */
                | (packed[6*(3-i-1)+7]);         /* LSB */

    corrs[i].I = sign.xtend; /* Sign extend! */
  }
}

/** Read data from a NAP tracking channel's CORR register.
 *
 * \param channel      NAP tracking channel whose CORR register to read.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void track_read_corr_blocking(u8 channel, u16* sample_count, corr_t corrs[])
{
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) + 16 bits sample count. */
  u8 temp[2*3*3+2];
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET, 2*3*3+2, temp, temp);
  track_unpack_corr(temp, sample_count, corrs);
}

/** Unpack data read from a NAP tracking channel's PHASE register.
 *
 * \param packed        Array of u8 data read from channnel's PHASE register.
 * \param carrier_phase Carrier phase at end of last correlation period.
 * \param code_phase    Prompt code phase at end of last correlation period.
 *                      (deprecated, is always zero, as prompt code phase
 *                       rollovers are defined to be edges of correlation
 *                       period)
 */
/* TODO : take code phase out of phase register, it's always zero */
void track_unpack_phase(u8 packed[], u32* carrier_phase, u64* code_phase)
{
  *carrier_phase = packed[8] |
                   (packed[7] << 8) |
                   (packed[6] << 16);
  *code_phase = (u64)packed[5] |
                ((u64)packed[4] << 8) |
                ((u64)packed[3] << 16) |
                ((u64)packed[2] << 24) |
                ((u64)packed[1] << 32) |
                ((u64)packed[0] << 40);
}

/** Read data from a NAP tracking channel's PHASE register.
 *
 * \param channel       NAP tracking channel whose PHASE register to read.
 * \param carrier_phase Carrier phase at end of last correlation period.
 * \param code_phase    Prompt code phase at end of last correlation period.
 *                      (deprecated, is always zero, as prompt code phase
 *                       rollovers are defined to be edges of correlation
 *                       period)
 */
void track_read_phase_blocking(u8 channel, u32* carrier_phase, u64* code_phase)
{
  u8 temp[9] = {0, 0, 0x22, 0, 0, 0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_PHASE_OFFSET, 9, temp, temp);
  track_unpack_phase(temp, carrier_phase, code_phase);
}

/** Write CA code to tracking channel's code ram.
 * CA Code for SV to be searched for must be written into channel's code ram
 * before acquisitions are started.
 *
 * \param prn PRN number (0-31) of CA code to be written.
 */
void track_write_code_blocking(u8 channel,u8 prn)
{
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CODE_OFFSET, 128, 0, ca_code(prn));
}

/** Set the LOAD ENABLE bit of the NAP CW channel's LOAD register.
 * When the LOAD ENABLE bit is set, the CW channel will start loading samples
 * into its sample ram, starting at the first sample clock cycle after the
 * NAP's internal timing strobe goes low. Writing to the LOAD register will
 * clear the CW_LOAD interrupt.
 */
void cw_set_load_enable_blocking()
{
  u8 temp[1] = {0xFF};
  swift_nap_xfer_blocking(SPI_ID_CW_LOAD_ENABLE, 1, 0, temp);
}

/** Clear the LOAD ENABLE bit of the NAP CW channel's LOAD register.
 * After a load to the CW channel's sample ram, the LOAD ENABLE bit must be
 * cleared, or future timing strobes will cause the ram to be re-loaded.
 * Writing to the LOAD register will clear the CW_LOAD interrupt.
 */
void cw_clear_load_enable_blocking()
{
  u8 temp[1] = {0x00};
  swift_nap_xfer_blocking(SPI_ID_CW_LOAD_ENABLE, 1, 0, temp);
}

/** Pack data for writing to NAP CW channel INIT register.
 *
 * \param prn          PRN number - (0..31) (deprecated)
 * \param carrier_freq CW frequency in CW INIT register units.
 */
void cw_pack_init(u8 pack[], s32 carrier_freq)
{
  pack[0] = (1<<3) |                        /* cw enabled */
            ((carrier_freq >> 30) & 0x04) | /* carrier freq [sign] */
            ((carrier_freq >> 16) & 0x03);  /* carrier freq [17:16] */
  pack[1] = (carrier_freq >> 8) & 0xFF;     /* carrier freq [15:8] */
  pack[2] = carrier_freq & 0xFF;            /* carrier freq [7:0] */
}

/** Write CW parameters to NAP CW channel's INIT register.
 * If the channel is currently disabled, it will be enabled and start a CW
 * search with these parameters. If it is currently enabled, another CW search
 * will start with these parameters when the current CW search finishes.
 * NOTE: If searching more than one spectrum point, the second set of CW search
 * parameters should be written into the channel as soon as possible after the
 * first set, as they are pipelined and used immediately after the first
 * CW correlation finishes. If only searching one point, cw_disable_blocking
 * should be called as soon as possible after the first set of CW parameters
 * are written, and again after the CW_DONE interrupt occurs to clear the
 * interrupt.
 *
 * \param carrier_freq CW frequency in CW INIT register units.
 */
void cw_write_init_blocking(s32 carrier_freq)
{
  u8 temp[3];
  cw_pack_init(temp, carrier_freq);
  swift_nap_xfer_blocking(SPI_ID_CW_INIT, 3, 0, temp);
}

/** Disable NAP CW channel.
 * Write to the CW channel's INIT register to disable the correlations (clears
 * enable bit). This must be written once to pipeline disable the correlations,
 * and then a second time to clear the CW_DONE IRQ after the last correlation
 * has finished.
 */
void cw_disable_blocking()
{
  u8 temp[3] = {0,0,0};
  swift_nap_xfer_blocking(SPI_ID_CW_INIT, 3, 0, temp);
}

/** Unpack correlations read from CW channel.
 *
 * \param packed Array of u8 data read from NAP CW channel CORR register.
 * \param corrs  Pointer to single corr_t.
 */
void cw_unpack_corr(u8 packed[], corr_t* corrs)
{

	/* should 24 instead be a macro constant? */
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */
  struct {s32 xtend:24;} sign;

  sign.xtend  = (packed[0] << 16) /* MSB */
              | (packed[1] << 8)  /* Middle byte */
              | (packed[2]);      /* LSB */

  corrs->Q = sign.xtend; /* Sign extend! */

  sign.xtend  = (packed[3] << 16) /* MSB */
              | (packed[4] << 8)  /* Middle byte */
              | (packed[5]);      /* LSB */

  corrs->I = sign.xtend; /* Sign extend! */
}

/** Read correlations from CW channel.
 * Must be called after the NAP IRQ register IRQ_CW_DONE bit goes high, before
 * the next CW cycle is complete.
 *
 * \param corrs Pointer to single corr_t.
 */
void cw_read_corr_blocking(corr_t* corrs)
{
  u8 temp[6]; /* 6 u8 = 48 bits = 2*(24 bits) */
  cw_unpack_corr(temp,corrs);
  swift_nap_xfer_blocking(SPI_ID_CW_CORR, 6, temp, temp);
}

/** Return status of NAP authentication hash comparison.
 * Status macros are defined in swift_nap_io.h.
 *
 * \return Status of NAP authentication hash comparison.
 */
u8 get_nap_hash_status()
{
  u8 temp[1] = {0};
  swift_nap_xfer_blocking(SPI_ID_HASH_STATUS,1,temp,temp);
  return temp[0];
}

/** Return git commit hash from NAP configuration build.
 * Retrieves commit hash of HDL repository at the time FPGA configuration was
 * built.
 *
 * \param git_hash Array of u8 (length 20) in which git hash will be put.
 */
void get_nap_git_hash(u8 git_hash[])
{
  m25_read(FLASH_NAP_GIT_HASH_ADDR, 20, git_hash);
}

/** Return git repository cleanliness status from NAP configuration build.
 * Retrieves cleanliness status of HDL repository at the time FPGA
 * configuration was built.
 *
 * \return 1 if repository was unclean, 0 if clean
 */
u8 get_nap_git_unclean()
{
  u8 unclean;
  m25_read(FLASH_NAP_GIT_UNCLEAN_ADDR, 1, &unclean);
  return unclean;
}

/** SPI DMA is not implemented yet. */
void spi_dma_setup() /* not yet updated for v2.3 */
{
  /* Enable clock to DMA peripheral. */
  RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Set appropriate bits in SPI2_CR2 to pass DMA requests to DMA controller. */
  spi_enable_rx_dma(SPI2);
  spi_enable_tx_dma(SPI2);

  /* Make sure stream is disabled to start. */
  DMA1_S3CR &= ~DMA_SxCR_EN;

  /* RM0090 - 9.3.17 : Supposed to wait until enable bit reads '0' before we
   * write to registers. */
  while (DMA1_S3CR & DMA_SxCR_EN);

  /* SPI2 RX - stream 3, channel 0, high priority */
  DMA1_S3CR =
    /* Error interrupts. */
    DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
    /* Transfer complete interrupt. */
    DMA_SxCR_TCIE |
    DMA_SxCR_DIR_PERIPHERAL_TO_MEM |
    /* Increment the memory address after each transfer. */
    DMA_SxCR_MINC |
    /* 8 bit transfers from SPI peripheral. */
    DMA_SxCR_PSIZE_8BIT |
    /* and to memory. */
    DMA_SxCR_MSIZE_8BIT |
    /* High priority. */
    DMA_SxCR_PL_HIGH |
    /* The channel selects which request line will trigger a transfer.
     * In this case, channel 0 = SPI2_RX (see CD00225773.pdf Table 22).
     */
    DMA_SxCR_CHSEL(0);

  /* Don't transfer any data yet (will be set in the initiating function) */
  DMA1_S3NDTR = 0;

  /* DMA from the SPI2 data register... */
  DMA1_S3PAR = &SPI2_DR;
  /* ...to the SPI RX buffer. */
  DMA1_S3M0AR = spi_dma_buffer;

  /* FIFO disabled, i.e. direct mode. TODO: see if FIFO helps performance. */
  DMA1_S3FCR = 0;

  /* SPI2 TX - stream 4, channel 0, high priority */
  DMA1_S4CR = 0; /* Make sure stream is disabled to start. */

  /* Make sure stream is disabled to start. */
  DMA1_S4CR &= ~DMA_SxCR_EN;

  /* RM0090 - 9.3.17 : Supposed to wait until enable bit reads '0' before we
   * write to registers. */
  while (DMA1_S4CR & DMA_SxCR_EN);

  /* SPI2 TX - stream 4, channel 0, high priority */
  DMA1_S4CR =
    /* Error interrupts. */
    DMA_SxCR_DMEIE | DMA_SxCR_TEIE |
    /* No transfer complete interrupt (we'll use the RX one) */
    DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
    /* Increment the memory address after each transfer. */
    DMA_SxCR_MINC |
    /* 8 bit transfers to SPI peripheral... */
    DMA_SxCR_PSIZE_8BIT |
    /* ...and from memory. */
    DMA_SxCR_MSIZE_8BIT |
    /* High priority. */
    DMA_SxCR_PL_HIGH |
    /* The channel selects which request line will trigger a transfer.
     * In this case, channel 0 = SPI2_TX (see CD00225773.pdf Table 22).
     */
    DMA_SxCR_CHSEL(0);

  /* For now, don't transfer any number of datas (will be set in the initiating function). */
  DMA1_S4NDTR = 0;

  /* Stream data to the SPI2 data register... */
  DMA1_S4PAR = &SPI2_DR;
  /* ...from the SPI TX buffer. */
  DMA1_S4M0AR = spi_dma_buffer;

  /* FIFO disabled, i.e. direct mode. TODO: see if FIFO helps performance */
  DMA1_S4FCR = 0;

}

/** SPI DMA is not implemented yet. */
void swift_nap_xfer_dma(u8 n_bytes) /* not yet updated for v2.3 */
{

  if (DMA1_S3CR & DMA_SxCR_EN || DMA1_S4CR & DMA_SxCR_EN) {
    /* DMA transfer already in progress.
     * TODO: handle this gracefully, but for now...
     */
    speaking_death("SPI DMA xfer already in progess");
  }

  DMA1_S4NDTR = n_bytes;

  spi_slave_select(SPI_SLAVE_FPGA);

  /* Enable DMA channels. */
  DMA1_S3CR |= DMA_SxCR_EN;
  DMA1_S4CR |= DMA_SxCR_EN;

  while (DMA1_S4NDTR > 0);
}
