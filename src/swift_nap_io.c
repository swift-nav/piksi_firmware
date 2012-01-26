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

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/timer.h>

#include "swift_nap_io.h"
#include "track.h"
#include "hw/spi.h"
#include "hw/max2769.h"

u32 exti_count = 0;

void swift_nap_setup()
{
  /* Initialise the SPI peripheral. */
  spi_setup();
  /* Setup the front end. */
  max2769_setup();

  /* Setup the reset line GPIO. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
  gpio_clear(GPIOB, GPIO6);

  /* Setup the timing strobe output. */
  timing_strobe_setup();

  /* Setup the external interrupts. */
  exti_setup();
}

void swift_nap_reset()
{
  gpio_set(GPIOB, GPIO6);
  for (int i = 0; i < 50; i++)
    __asm__("nop");
  gpio_clear(GPIOB, GPIO6);
}

void swift_nap_xfer_blocking(u8 spi_id, u8 n_bytes, u8 data_in[], u8 data_out[])
{
  spi_slave_select(SPI_SLAVE_FPGA);

  spi_xfer(SPI_BUS_FPGA, spi_id);

  /* If data_in is NULL then discard read data. */
  if (data_in)
    for (u8 i=n_bytes; i>0; i--)
      data_in[n_bytes-i] = spi_xfer(SPI_BUS_FPGA, data_out[i-1]);
  else
    for (u8 i=n_bytes; i>0; i--)
    //for (u8 i=0; i<n_bytes; i++)
      spi_xfer(SPI_BUS_FPGA, data_out[i-1]);

  spi_slave_deselect();
}

void exti_setup()
{
  /* Signal from the FPGA is on PC6. */

  /* Enable clock to GPIOA. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  /* Enable clock to SYSCFG which contains the EXTI functionality. */
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  exti_select_source(EXTI6, GPIOC);
	exti_set_trigger(EXTI6, EXTI_TRIGGER_RISING);
  exti_reset_request(EXTI6);
	exti_enable_request(EXTI6);

	/* Enable EXTI6 interrupt */
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

void exti9_5_isr()
{
  exti_reset_request(EXTI6);

  u32 irq = swift_nap_read_irq_blocking();

  if (irq & IRQ_TRACK) {
    tracking_channel_get_corrs(0);
  }
  
  exti_count++;
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
  timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FORCE_HIGH);
  timer_set_oc_value(TIM2, TIM_OC1, falling_edge_count);
  timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_INACTIVE);
}

u32 timing_count() {
  return TIM2_CNT;
}

void timing_strobe_setup()
{
  /* Setup Timer 2 as out global sample counter. */
  RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Set timer prescale to divide APB bus back to the sample
   * clock frequency.
   *
   * NOTE: This will only work for ppre1_freq that is an integer 
   *       multiple of the sample clock.
   * NOTE: Assumes APB1 prescale != 1, see Ref Man pg. 84
   */
  timer_set_prescaler(TIM2, rcc_ppre1_frequency / SAMPLE_FREQ);

  /* Set time auto-reload value to the longest possible period. */
  timer_set_period(TIM2, 0xFFFFFFFF);

  /* Configure PA0 as TIM2CH1 alternate function. */
  timer_enable_oc_output(TIM2, TIM_OC1);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO0);

  /* Enable timer */
  TIM2_CNT = 0;
  timer_generate_event(TIM2, TIM_EGR_UG);
  timer_enable_counter(TIM2);
}

u32 swift_nap_read_irq_blocking()
{
  u8 temp = 0;
  swift_nap_xfer_blocking(SPI_ID_IRQ, 1, &temp, &temp); 
  return temp;
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

/** Write initialisation parameters to the Swift NAP acquisition channel.
 * Writes acquisition initialisation parameters into the ACQ_INIT
 * register on the Swift NAP.
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
void acq_write_init_blocking(u8 prn, u16 code_phase, s16 carrier_freq)
{
  u32 temp = 0;

  /* Modulo 1023*4 in case adding ACQ_N_TAPS-1 rolls us over a
   * code phase boundary.
   */
  u16 code_phase_reg_value = (code_phase+ACQ_N_TAPS-1) % (1023*4);

  temp |= prn & 0x1F;
  /* Write corrected code phase, see note above. */
  temp |= ((u32)code_phase_reg_value << 5) & 0x1FFE0;
  temp |= ((u32)carrier_freq << 17) & 0x1FFE0000;
  temp |= (1 << 29); /* Acq enabled */

  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}

/** Disable the acquisition channel.
 * Writes to the ACQ_INIT register in the Swift NAP to
 * disable the acquisition channel.
 */
void acq_disable_blocking()
{
  u32 temp = 0;
  swift_nap_xfer_blocking(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}

void acq_read_corr_blocking(corr_t corrs[]) {
  u8 temp[2*ACQ_N_TAPS * 3];
  
  swift_nap_xfer_blocking(SPI_ID_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);

  struct {s32 x:24;} s;
  for (u8 i=0; i<ACQ_N_TAPS; i++) {
    corrs[i].Q  = (u32)temp[6*i+2];
    corrs[i].Q |= (u32)temp[6*i+1] << 8;
    corrs[i].Q |= (u32)temp[6*i]   << 16;
    corrs[i].Q = s.x = corrs[i].Q; /* Sign extend! */

    corrs[i].I  = (u32)temp[6*i+5];
    corrs[i].I |= (u32)temp[6*i+4] << 8;
    corrs[i].I |= (u32)temp[6*i+3] << 16;
    corrs[i].I = s.x = corrs[i].I; /* Sign extend! */
  }
}

void track_write_init_blocking(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase) {
  /* for length(prn) = 5,
   *     length(carrier_phase) = 24,
   *     length(code_phase) = 14
   */
  u8 temp[6] = {0, 0, 0, 0, 0, 0};

  temp[0] = (prn & 0x1F) | (carrier_phase << 5 & 0xE0);
  temp[1] = (carrier_phase << 5) >> 8;
  temp[2] = (carrier_phase << 5) >> 16;
  temp[3] = (((carrier_phase << 5) >> 24) & 0x1F) | (code_phase << 5);
  temp[4] = (code_phase << 5) >> 8;
  temp[5] = ((code_phase << 5) >> 16) & 0x07;

  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_INIT_OFFSET, 6, 0, temp);
}

void track_write_update_blocking(u8 channel, s32 carrier_freq, u32 code_phase_rate) {
  u8 temp[6] = {0, 0, 0, 0, 0, 0};

  temp[0] = carrier_freq;
  temp[1] = (carrier_freq >> 8);
  temp[2] = code_phase_rate;
  temp[3] = (code_phase_rate >> 8);
  temp[4] = (code_phase_rate >> 16);
  temp[5] = (code_phase_rate >> 24) & 0x1F;

  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_UPDATE_OFFSET, 6, 0, temp);
}

void track_read_corr_blocking(u8 channel, corr_t corrs[]) {
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) */
  u8 temp[2*3 * 3];
  
  swift_nap_xfer_blocking(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET, 2*3*3, temp, temp);

  struct {s32 x:24;} s;
  for (u8 i=0; i<3; i++) {
    corrs[2-i].Q  = (u32)temp[6*i+2];
    corrs[2-i].Q |= (u32)temp[6*i+1] << 8;
    corrs[2-i].Q |= (u32)temp[6*i]   << 16;
    corrs[2-i].Q = s.x = corrs[2-i].Q; /* Sign extend! */

    corrs[2-i].I  = (u32)temp[6*i+5];
    corrs[2-i].I |= (u32)temp[6*i+4] << 8;
    corrs[2-i].I |= (u32)temp[6*i+3] << 16;
    corrs[2-i].I = s.x = corrs[2-i].I; /* Sign extend! */
  }
}

