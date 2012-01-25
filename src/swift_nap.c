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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/timer.h>

#include "swift_nap.h"
#include "board/spi.h"
#include "board/exti.h"

void swift_nap_setup()
{
  /* Setup the reset line GPIO. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
  gpio_clear(GPIOB, GPIO6);

  /* Setup the timing strobe output. */
  timing_strobe_setup();
}

void swift_nap_reset()
{
  gpio_set(GPIOB, GPIO6);
  for (int i = 0; i < 50; i++)
    __asm__("nop");
  gpio_clear(GPIOB, GPIO6);
}

void swift_nap_xfer(u8 spi_id, u8 n_bytes, u8 data_in[], u8 data_out[])
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

void acq_set_load_enable() {
  u8 temp[1] = {0xFF};
  swift_nap_xfer(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp); 
}

void acq_clear_load_enable() {
  u8 temp[1] = {0x00};
  swift_nap_xfer(SPI_ID_ACQ_LOAD_ENABLE, 1, 0, temp); 
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
void acq_write_init(u8 prn, u16 code_phase, s16 carrier_freq)
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

  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}

/** Disable the acquisition channel.
 * Writes to the ACQ_INIT register in the Swift NAP to
 * disable the acquisition channel.
 */
void acq_disable()
{
  u32 temp = 0;
  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}

void acq_read_corr(corr_t corrs[]) {
  u8 temp[2*ACQ_N_TAPS * 3];
  
  swift_nap_xfer(SPI_ID_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);

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

void do_one_acq(u8 prn, u16 code_phase, s16 carrier_freq, corr_t corrs[]) {
  acq_write_init(prn, code_phase, carrier_freq); 

  /* Disable acq on next cycle, after this one has finished. */
  acq_disable();

  /* Wait for acq done IRQ. */
  wait_for_exti();

  /* Write to clear IRQ. */
  acq_disable();

  /* Read in correlations. */
  acq_read_corr(corrs);
}

s32 round_up(s32 n, s32 multiple) 
{ 
 s32 remainder = n % multiple;
 if (remainder == 0)
  return n;
 return n + multiple - remainder;
} 

/** Perform an aqcuisition.
 * Perform an acquisition for one PRN over a defined code and doppler range. Returns
 * the code phase and carrier frequency of the largest peak in the search space together
 * with the "SNR" value for that peak defined as (peak_magnitude - mean) / std_deviation.
 *
 * \param prn    PRN number - 1 (0..31) to attempt to aqcuire.
 * \param cp_min Lower bound for code phase search range in chips.
 * \param cp_max Upper bound for code phase search range in chips.
 * \param cf_min Lower bound for carrier freq. search range in Hz.
 * \param cf_max Upper bound for carrier freq. search range in Hz.
 * \param cp     Pointer to a float where the peak's code phase value will be stored in chips.
 * \param cf     Pointer to a float where the peak's carrier frequency will be stored in Hz.
 * \param snr    Pointer to a float where the "SNR" of the peak will be stored.
 */
void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr)
{
  gpio_set(GPIOC, GPIO11);

  corr_t cs[ACQ_N_TAPS];

  float mag, mag_sq, best_mag = 0;
  float sd, mean = 0, sq_mean = 0;
  u32 count = 0;
  u16 best_cp = 0;
  s16 best_cf = 0;

  /* Calculate the range parameters in acq units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
  s16 cf_step = cf_bin_width*ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  s16 cf_min_acq = cf_step*floor(cf_min*ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)cf_step);
  s16 cf_max_acq = cf_step*ceil(cf_max*ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)cf_step);
  /* cp_step = ACQ_N_TAPS */
  u16 cp_min_acq = ACQ_N_TAPS*floor(cp_min*ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)ACQ_N_TAPS);
  u16 cp_max_acq = ACQ_N_TAPS*ceil(cp_max*ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)ACQ_N_TAPS);

  /* Write first and second sets of acq parameters (for pipelining). */
  acq_write_init(prn, cp_min_acq, cf_min_acq); 
  /* If we are only doing a single acq then write disable here. */
  /*printf("(%d, %d) => \n", cp_min_acq+ACQ_N_TAPS, cf_min_acq);*/
  /*if (cp_min_acq+ACQ_N_TAPS >= cp_max_acq && cf_min_acq+cf_step >= cf_max_acq) {*/
          /*printf(" D\n");*/
    /*acq_disable();*/
  /*}else*/
    acq_write_init(prn, cp_min_acq+ACQ_N_TAPS, cf_min_acq); 

  /* Save the exti count so we can detect the next interrupt. */
  u32 last_last_exti = last_exti_count();

  for (s16 carrier_freq = cf_min_acq; carrier_freq <= cf_max_acq; carrier_freq += cf_step) {
    for (u16 code_phase = cp_min_acq; code_phase < cp_max_acq; code_phase += ACQ_N_TAPS) {

      /* Wait for acq done IRQ and save the new exti count. */
      while(last_exti_count() == last_last_exti);
      last_last_exti = last_exti_count();

      /* Read in correlations. */
      acq_read_corr(cs);

      /* Write parameters for 2 cycles time for acq pipelining apart
       * from the last two cycles where we want to write disable.
       * The first time to disable and the second time really just
       * to clear the interrupt from the last cycle.
       *
       * NOTE: we must take care to handle wrapping, when we get to
       * the end of the code phase range the parameters for 2 cycles
       * time will be with the next carrier freq value and a small
       * code phase value.
       */
      if (code_phase < cp_max_acq - 2*ACQ_N_TAPS) {
        /*printf("(%d, %d) => %d, %d\n", code_phase, carrier_freq, code_phase+2*ACQ_N_TAPS, carrier_freq);*/
        acq_write_init(prn, code_phase+2*ACQ_N_TAPS, carrier_freq); 
      } else {
        if (carrier_freq >= cf_max_acq && code_phase >= (cp_max_acq-2*ACQ_N_TAPS)) {
          gpio_set(GPIOC, GPIO10);
          /*printf("(%d, %d) => D\n", code_phase, carrier_freq);*/
          acq_disable();
          gpio_clear(GPIOC, GPIO10);
        } else {
          /*printf("(%d, %d) => %d, %d\n", code_phase, carrier_freq, cp_min_acq+code_phase-cp_max_acq+2*ACQ_N_TAPS, carrier_freq+cf_step);*/
          acq_write_init(prn, cp_min_acq + code_phase - cp_max_acq + 2*ACQ_N_TAPS, carrier_freq+cf_step); 
        }
      }

      for (u8 i=0; i<ACQ_N_TAPS; i++) {
        mag_sq = (float)cs[i].I*(float)cs[i].I + (float)cs[i].Q*(float)cs[i].Q;
        mag = sqrt(mag_sq);
        mean += mag;
        sq_mean += mag_sq;
        if (mag > best_mag) {
          best_mag = mag;
          best_cf = carrier_freq;
          best_cp = code_phase + i;
        }
        /*printf("%f, %f, %f, %f, %f, %d, %d\n", mag_sq, mag, mean, sq_mean, best_mag, best_cf, best_cp);*/
      }
      count += ACQ_N_TAPS;
    }
  }

  sd = sqrt(count*sq_mean - mean*mean) / count;
  mean = mean / count;

  *cp = (float)best_cp / ACQ_CODE_PHASE_UNITS_PER_CHIP;
  *cf = (float)best_cf / ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  *snr = (best_mag - mean) / sd;

  gpio_clear(GPIOC, GPIO11);
}

void track_write_init(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase) {
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

  swift_nap_xfer(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_INIT_OFFSET, 6, 0, temp);
}

void track_write_update(u8 channel, s32 carrier_freq, u32 code_phase_rate) {
  u8 temp[6] = {0, 0, 0, 0, 0, 0};

  temp[0] = carrier_freq;
  temp[1] = (carrier_freq >> 8);
  temp[2] = code_phase_rate;
  temp[3] = (code_phase_rate >> 8);
  temp[4] = (code_phase_rate >> 16);
  temp[5] = (code_phase_rate >> 24) & 0x1F;

  swift_nap_xfer(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_UPDATE_OFFSET, 6, 0, temp);
}

void track_read_corr(u8 channel, corr_t corrs[]) {
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) */
  u8 temp[2*3 * 3];
  
  swift_nap_xfer(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET, 2*3*3, temp, temp);

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

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1.0 + carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;

  /* Internal Swift NAP code phase is in chips*2^32:
   *
   * |  Chip no.  | Sub-chip | Fractional sub-chip |
   * | 0 ... 1022 | 0 ... 15 |  0 ... (2^28 - 1)   |
   *
   * Code phase rate is directly added in this representation,
   * the nominal code phase rate corresponds to 1 sub-chip.
   */

  /* Calculate code phase in chips*2^32. */
  u64 propagated_code_phase = (u64)(code_phase * (((u64)1)<<32)) + n_samples * (u64)code_phase_rate;

  /* Convert code phase back to natural units with sub-chip precision.
   * NOTE: the modulo is required to fix the fact rollover should 
   * occur at 1023 not 1024.
   */
  return (float)((u32)(propagated_code_phase >> 28) % (1023*16)) / 16.0;
}

/** Initialises a tracking channel.
 * Initialises a tracking channel on the Swift NAP.
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param code_phase         Code phase at start of tracking in chips [0-1023).
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 */
void tracking_channel_init(u8 prn, u8 channel, float code_phase, float carrier_freq, u32 start_sample_count)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1 + carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;

  /* TODO: Write PRN into tracking channel when the FPGA code supports this. */

  /* Starting carrier phase is set to zero as we don't 
   * know the carrier freq well enough to calculate it.
   */
  track_write_init(channel, prn, 0, code_phase*TRACK_CODE_PHASE_UNITS_PER_CHIP);
  track_write_update(channel, carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, code_phase_rate);

  /* Schedule the timing strobe for start_sample_count. */
  timing_strobe(start_sample_count);
}

