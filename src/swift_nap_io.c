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

/* NAP Parameters stored in the FPGA configuration flash */
u8 ACQ_N_TAPS;
u8 TRACK_N_CHANNELS;

u32 exti_count = 0;

#define SPI_DMA_BUFFER_LEN 22
u8 spi_dma_buffer[SPI_DMA_BUFFER_LEN];

void swift_nap_callbacks_setup();

void get_nap_parameters();

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
   * (It uses the SPI2 bus for this.) */
  /* TODO: Timeout here? */
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

  /* Setup the timing strobe output. */
  //timing_strobe_setup();

  /* Setup the external interrupts. */
  exti_setup();

  /* Setup callback functions */
  swift_nap_callbacks_setup();

  /* Get NAP parameters (number of acquisition taps, number of tracking
   * channels, etc) from flash */
  get_nap_parameters();
}

void swift_nap_reset()
{
  gpio_set(GPIOA, GPIO2);
  for (int i = 0; i < 50; i++)
    __asm__("nop");
  gpio_clear(GPIOA, GPIO2);
  for (int i = 0; i < 200; i++)
    __asm__("nop");
}

/* Check if configuration is finished. Returns 1 if configuration is finished
 * (line high), 0 if not finished (line low) */
u8 swift_nap_conf_done()
{
  return ((gpio_port_read(GPIOC))>>1) & 0x01;
}

void swift_nap_conf_b_setup()
{
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set(GPIOC, GPIO12);
}

void swift_nap_conf_b_set()
{
  gpio_set(GPIOC, GPIO12);
}

void swift_nap_conf_b_clear()
{
  gpio_clear(GPIOC, GPIO12);
}

/* Check if FPGA has finished reading hash from configuration flash. Returns 1
 * if configuration is finished (line low), 0 if not finished (line high) */
u8 swift_nap_hash_rd_done()
{
  return ~(((gpio_port_read(GPIOA))>>3) | 0xFE);
}

void swift_nap_xfer_blocking(u8 spi_id, u16 n_bytes, u8 data_in[], const u8 data_out[])
{

  // Check that there's no DMA transfer in progress
  //if (DMA1_S3CR & DMA_SxCR_EN || DMA1_S4CR & DMA_SxCR_EN) {
    /* DMA transfer already in progress.
     * TODO: handle this gracefully, but for now...
     */
    //speaking_death("SPI DMA xfer already in progess");
  //}

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

    /* Test is the nth tracking irq flag is set, if so service it. */
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

u32 last_exti_count()
{
  return exti_count;
}

void wait_for_exti()
{
  u32 last_last_exti = last_exti_count();
  while(last_exti_count() == last_last_exti);
}

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

u32 timing_count(){
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_TIMING_COUNT,4,temp,temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

u32 timing_count_latched(){
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_TIMING_COUNT_LATCH,4,temp,temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

u32 swift_nap_read_irq_blocking()
{
  u8 temp[4] = {0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_IRQ, 4, temp, temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

u32 swift_nap_read_error_blocking()
{
  u8 temp[4] = {0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_ERROR, 4, temp, temp);
  return (temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|temp[3];
}

void acq_set_load_enable_blocking()
{
  u8 temp[1] = {0xFF};
  swift_nap_xfer_blocking(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp);
}

void acq_clear_load_enable_blocking()
{
  u8 temp[1] = {0x00};
  swift_nap_xfer_blocking(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp);
}

/* Pack data for writing to NAP acq init register
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
 * \param prn          PRN number - 1 (0..31)
 * \param code_phase   Code phase of the first correlation returned
 *                     (see note above), in acquisition units.
 * \param carrier_freq Carrier frequency i.e. Doppler in acquisition
 *                     units.
 */
void acq_pack_init(u8 pack[], u8 prn, u16 code_phase, s16 carrier_freq){

  /* Modulo 1023*4 in case adding ACQ_N_TAPS-1 rolls us over a
   * code phase boundary.
   */
  u16 code_phase_reg_value = (code_phase+ACQ_N_TAPS-1) % (1023*4);

  pack[0] = (1<<5) |                      // Acq enabled
            ((carrier_freq >> 7) & 0x1F); // carrier freq [11:7]

  pack[1] = (carrier_freq << 1) |         // carrier freq [6:0]
            (code_phase_reg_value >> 11); // code phase [11]

  pack[2] = code_phase_reg_value >> 3;    // code phase [10:3]

  pack[3] = (code_phase_reg_value << 5) | // code phase [2:0]
            (prn & 0x1F);                 // PRN number (0..31)
}

/** Write initialisation parameters to the Swift NAP acquisition channel.
 * Writes acquisition initialisation parameters into the ACQ_INIT
 * register on the Swift NAP.
  */
void acq_write_init_blocking(u8 prn, u16 code_phase, s16 carrier_freq)
{
  u8 temp[4];
  acq_pack_init(temp, prn, code_phase, carrier_freq);
  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, temp);
}

/** Disable the acquisition channel.
 * Writes to the ACQ_INIT register in the Swift NAP to
 * disable the acquisition channel.
 */
void acq_disable_blocking()
{
  u8 temp[4] = {0,0,0,0};
  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, temp);
}

/* Unpack correlations read from acquisition channel */
void acq_unpack_corr(u8 packed[], corr_t corrs[]) {

  struct {s32 xtend:24;} sign; // graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend

  for (u8 i=0; i<ACQ_N_TAPS; i++) {

    sign.xtend  = (packed[6*i]   << 16)    // MSB
                | (packed[6*i+1] << 8)     // Middle byte
                | (packed[6*i+2]);         // LSB

    corrs[i].Q = sign.xtend; /* Sign extend! */

    sign.xtend  = (packed[6*i+3] << 16)    // MSB
                | (packed[6*i+4] << 8)     // Middle byte
                | (packed[6*i+5]);         // LSB

    corrs[i].I = sign.xtend; /* Sign extend! */
  }
}

void acq_read_corr_blocking(corr_t corrs[]) {
  u8 temp[2*ACQ_N_TAPS * 3];
  swift_nap_xfer_blocking(SPI_ID_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);
  acq_unpack_corr(temp, corrs);
}

void acq_write_code_blocking(u8 prn) {
  swift_nap_xfer_blocking(SPI_ID_ACQ_CODE, 128, 0, ca_code(prn));
}

void track_pack_init(u8 pack[], u8 prn, s32 carrier_phase, u16 code_phase){
  /* for length(prn) = 5,
   *     length(carrier_phase) = 24,
   *     length(code_phase) = 14
   */
  pack[0] = ((code_phase << 5) >> 16) & 0x07;
  pack[1] = (code_phase << 5) >> 8;
  pack[2] = (((carrier_phase << 5) >> 24) & 0x1F) | (code_phase << 5);
  pack[3] = (carrier_phase << 5) >> 16;
  pack[4] = (carrier_phase << 5) >> 8;
  pack[5] = (prn & 0x1F) | (carrier_phase << 5 & 0xE0);
}

void track_write_init_blocking(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase) {
  u8 temp[6] = {0, 0, 0, 0, 0, 0};
  track_pack_init(temp, prn, carrier_phase, code_phase);
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_INIT_OFFSET, 6, 0, temp);
}

void track_pack_update(u8 pack[], s32 carrier_freq, u32 code_phase_rate){
  /* for length(prn) = 5,
   *     length(carrier_phase) = 24,
   *     length(code_phase) = 14
   */
  pack[0] = (code_phase_rate >> 24) & 0x1F;
  pack[1] = (code_phase_rate >> 16);
  pack[2] = (code_phase_rate >> 8);
  pack[3] = code_phase_rate;
  pack[4] = (carrier_freq >> 8);
  pack[5] = carrier_freq;
}

void track_write_update_blocking(u8 channel, s32 carrier_freq, u32 code_phase_rate) {
  u8 temp[6] = {0, 0, 0, 0, 0, 0};
  track_pack_update(temp, carrier_freq, code_phase_rate);
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_UPDATE_OFFSET, 6, 0, temp);
}

void track_unpack_corr(u8 packed[], u16* sample_count, corr_t corrs[]) {

  struct {s32 xtend:24;} sign; // graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend

  *sample_count = (packed[0]<<8) | packed[1];

  for (u8 i=0; i<3; i++) {

    sign.xtend  = (packed[6*(3-i-1)+2] << 16)    // MSB
                | (packed[6*(3-i-1)+3] << 8)     // Middle byte
                | (packed[6*(3-i-1)+4]);         // LSB

    corrs[i].Q = sign.xtend; /* Sign extend! */

    sign.xtend  = (packed[6*(3-i-1)+5] << 16)    // MSB
                | (packed[6*(3-i-1)+6] << 8)     // Middle byte
                | (packed[6*(3-i-1)+7]);         // LSB

    corrs[i].I = sign.xtend; /* Sign extend! */
  }
}

void track_read_corr_blocking(u8 channel, u16* sample_count, corr_t corrs[]) {
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8)
   * + 16 bits sample count.
   */
  u8 temp[2*3*3+2];
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET, 2*3*3+2, temp, temp);
  track_unpack_corr(temp, sample_count, corrs);
}

void track_unpack_phase(u8 packed[], u32* carrier_phase, u64* code_phase){
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

void track_read_phase_blocking(u8 channel, u32* carrier_phase, u64* code_phase)
{
  u8 temp[9] = {0, 0, 0x22, 0, 0, 0, 0, 0, 0};
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_PHASE_OFFSET, 9, temp, temp);
  track_unpack_phase(temp, carrier_phase, code_phase);
}

void spi_dma_setup() { // not yet updated for v2.2

  RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;  // Enable clock to DMA peripheral

  spi_enable_rx_dma(SPI2);  // Set appropriate bits in SPI2_CR2 to pass DMA requests to DMA controller
  spi_enable_tx_dma(SPI2);

  /* SPI2 RX - stream 3, channel 0, high priority*/
  DMA1_S3CR = 0; /* Make sure stream is disabled to start. */
  DMA1_S3CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |  // Error interrupts
              DMA_SxCR_TCIE |                      // Transfer complete interrupt
              DMA_SxCR_DIR_PERIPHERAL_TO_MEM |
              DMA_SxCR_MINC |                         // Increment the memory address after each transfer
              DMA_SxCR_PSIZE_8BIT |                   // 8 bit transfers from SPI peripheral
              DMA_SxCR_MSIZE_8BIT |                   // and to memory
              DMA_SxCR_PL_HIGH |                      // High priority
              DMA_SxCR_CHSEL(0);                      // The channel selects which request line will trigger a transfer
                                                      // In this case, channel 0 = SPI2_RX (see CD00225773.pdf Table 22)

  DMA1_S3NDTR = 0;        // For now, don't transfer any number of datas (will be set in the initiating function)

  DMA1_S3PAR = &SPI2_DR;          // This is the address the data will be streamed from
  DMA1_S3M0AR = spi_dma_buffer;   // And where it's going to end up

  DMA1_S3FCR = 0;         // FIFO disabled, i.e. direct mode.  TODO: see if FIFO helps performance

  /* SPI2 TX - stream 4, channel 0, high priority */
  DMA1_S4CR = 0; /* Make sure stream is disabled to start. */
  DMA1_S4CR = DMA_SxCR_DMEIE | DMA_SxCR_TEIE |  // Error interrupts
                                                      // No transfer complete interrupt (we'll use the RX one)
              DMA_SxCR_DIR_MEM_TO_PERIPHERAL |
              DMA_SxCR_MINC |                         // Increment the memory address after each transfer
              DMA_SxCR_PSIZE_8BIT |                   // 8 bit transfers to SPI peripheral
              DMA_SxCR_MSIZE_8BIT |                   // and from memory
              DMA_SxCR_PL_HIGH |                      // High priority
              DMA_SxCR_CHSEL(0);                      // The channel selects which request line will trigger a transfer
                                                      // In this case, channel 0 = SPI2_TX (see CD00225773.pdf Table 22)

  DMA1_S4NDTR = 0;        // For now, don't transfer any number of datas (will be set in the initiating function)

  DMA1_S4PAR = &SPI2_DR;          // This is the address the data will be streamed to
  DMA1_S4M0AR = spi_dma_buffer;   // And where it's coming from

  DMA1_S4FCR = 0;         // FIFO disabled, i.e. direct mode.  TODO: see if FIFO helps performance
  /* SPI2 TX */

}

void swift_nap_xfer_dma(u8 n_bytes) { // not yet updated for v2.2

  if (DMA1_S3CR & DMA_SxCR_EN || DMA1_S4CR & DMA_SxCR_EN) {
    /* DMA transfer already in progress.
     * TODO: handle this gracefully, but for now...
     */
    speaking_death("SPI DMA xfer already in progess");
  }

  DMA1_S4NDTR = n_bytes; // For now, don't transfer any number of datas (will be set in the initiating function)

  spi_slave_select(SPI_SLAVE_FPGA);

  /* Enable DMA channels. */
  DMA1_S3CR |= DMA_SxCR_EN;
  DMA1_S4CR |= DMA_SxCR_EN;

  while (DMA1_S4NDTR > 0);
}

void track_read_corr_dma(u8 channel) // not yet updated for v2.2
{
  spi_dma_buffer[0] = SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET; // Select correlation result register

  /* Start 18 byte DMA xfer i.e. 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) */
  swift_nap_xfer_dma(2*3*3);
}

void track_unpack_corr_dma(corr_t corrs[]) // not yet updated for v2.2
{
  /* Correlations now in DMA buffer, skip first byte that
   * corresponds to the SPI ID.
   */
  u8* temp = spi_dma_buffer+1;

  struct {s32 x:24;} s;
  for (u8 i=0; i<3; i++) {
    corrs[2-i].Q  = (u32)temp[6*i+2];
    corrs[2-i].Q |= (u32)temp[6*i+1] << 8;
    corrs[2-i].Q |= (u32)temp[6*i]   << 16;
    corrs[2-i].Q = s.x = corrs[2-i].Q; // Sign extend!

    corrs[2-i].I  = (u32)temp[6*i+5];
    corrs[2-i].I |= (u32)temp[6*i+4] << 8;
    corrs[2-i].I |= (u32)temp[6*i+3] << 16;
    corrs[2-i].I = s.x = corrs[2-i].I; // Sign extend!
  }
}

void track_write_code_blocking(u8 channel,u8 prn) {
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CODE_OFFSET, 128, 0, ca_code(prn));
}

void cw_set_load_enable_blocking()
{
  u8 temp[1] = {0xFF};
  swift_nap_xfer_blocking(SPI_ID_CW_LOAD_ENABLE, 1, 0, temp);
}

void cw_clear_load_enable_blocking()
{
  u8 temp[1] = {0x00};
  swift_nap_xfer_blocking(SPI_ID_CW_LOAD_ENABLE, 1, 0, temp);
}

void cw_pack_init(u8 pack[], s32 carrier_freq){
  pack[0] = (1<<3) |                        // cw enabled
            ((carrier_freq >> 30) & 0x04) | // carrier freq [sign]
            ((carrier_freq >> 16) & 0x03);  // carrier freq [17:16]
  pack[1] = (carrier_freq >> 8) & 0xFF;     // carrier freq [15:8]
  pack[2] = carrier_freq & 0xFF;            // carrier freq [7:0]
}

void cw_write_init_blocking(s32 carrier_freq)
{
  u8 temp[3];
  cw_pack_init(temp, carrier_freq);
  swift_nap_xfer_blocking(SPI_ID_CW_INIT, 3, 0, temp);
}

void cw_disable_blocking()
{
  u8 temp[3] = {0,0,0};
  swift_nap_xfer_blocking(SPI_ID_CW_INIT, 3, 0, temp);
}

void cw_unpack_corr(u8 packed[], corr_t* corrs) {

	//should 24 instead be a macro constant?
  struct {s32 xtend:24;} sign; // graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend

  sign.xtend  = (packed[0] << 16)    // MSB
              | (packed[1] << 8)     // Middle byte
              | (packed[2]);         // LSB

  corrs->Q = sign.xtend; /* Sign extend! */

  sign.xtend  = (packed[3] << 16)    // MSB
              | (packed[4] << 8)     // Middle byte
              | (packed[5]);         // LSB

  corrs->I = sign.xtend; /* Sign extend! */
}

void cw_read_corr_blocking(corr_t* corrs) {
  u8 temp[6]; //6 u8 = 48 bits = 2*(24 bits)
  cw_unpack_corr(temp,corrs);
  swift_nap_xfer_blocking(SPI_ID_CW_CORR, 6, temp, temp);
}

/* Spartan 6 Device DNA is 57 bits, padded to 64 (with 0's) within FPGA */
void get_nap_dna(u8 dna[]){
  swift_nap_xfer_blocking(SPI_ID_DNA,8,dna,dna);
}

/* Returns status of device hash comparison inside FPGA 
 * 0x00 = hashes match
 * 0x01 = hashes do not match
 * 0x02 = one or more hashes are not ready to compare */
u8 get_nap_hash_status(){
  u8 temp[1] = {0};
  swift_nap_xfer_blocking(SPI_ID_HASH_STATUS,1,temp,temp);
  return temp[0];
}

void get_nap_dna_callback(){
  /* Retrieves Spartan 6 Device DNA and sends back over UART */
  u8 dna[8];
  get_nap_dna(dna);
  /* TODO : error handling for debug_send_msg failure? */
  debug_send_msg(MSG_NAP_DEVICE_DNA, 8, dna);
}

void swift_nap_callbacks_setup(){
  static msg_callbacks_node_t swift_nap_dna_node;
  debug_register_callback(MSG_NAP_DEVICE_DNA, &get_nap_dna_callback, &swift_nap_dna_node);
}

void get_nap_parameters(){
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

void get_nap_git_hash(u8 git_hash[]){
  m25_read(FLASH_NAP_GIT_HASH_ADDR,20,git_hash);
}

u8 get_nap_git_unclean(){
  u8 unclean;
  m25_read(FLASH_NAP_GIT_UNCLEAN_ADDR, 1, &unclean);
  return unclean;
}
