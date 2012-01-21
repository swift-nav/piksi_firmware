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

u32 acq_init(u8 enabled, u8 svid, u16 code_phase, s16 carrier_freq) {
  u32 temp = 0;
  u32 temp2 = 0;

  temp |= svid & 0x1F;
  temp |= ((u32)code_phase << 5) & 0x1FFE0;
  temp |= ((u32)carrier_freq << 17) & 0x1FFE0000;
  if (enabled)
    temp |= (1 << 29); /* Acq enabled */

  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, (u8*)&temp2, (u8*)&temp);

  return temp2;
}

void acq_disable() {
  u32 temp = 0;
  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}
/*
u32 unpack_22bits(u32 n, u8 A[]) {
  u8 s = 2;//4*(n+1) - 2; //((22*(n+1)) % 8) - 4;
  u32 i = 3*n;
  u32 r = 0;

  r |= (u32)A[i+2] >> s;
  r |= (u32)A[i+1] << (8 - s);
  r |= (u32)A[i+0] << (16 - s);
  r |= (u32)A[i-1] << (24 - s);
  r &= 0x3FFFFF; // 2^22 - 1
  return r;
}
*/

void acq_read_corr(corr_t corrs[]) {
  u8 temp[2*ACQ_N_TAPS * 3];
  
  swift_nap_xfer(SPI_ID_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);

  struct {s32 x:24;} s;
  for (u8 i=0; i<ACQ_N_TAPS; i++) {
    corrs[i].Q  = (u32)temp[6*i+2];
    corrs[i].Q |= (u32)temp[6*i+1] << 8;
    corrs[i].Q |= (u32)temp[6*i]   << 16;
    corrs[i].Q = s.x = corrs[i].Q; // Sign extend!

    corrs[i].I  = (u32)temp[6*i+5];
    corrs[i].I |= (u32)temp[6*i+4] << 8;
    corrs[i].I |= (u32)temp[6*i+3] << 16;
    corrs[i].I = s.x = corrs[i].I; // Sign extend!
  }
}

void do_one_acq(u8 svid, u16 code_phase, s16 carrier_freq, corr_t corrs[]) {
  acq_init(1, svid, code_phase, carrier_freq); 

  /*acq_init(0, svid, code_phase, carrier_freq); */
  acq_disable(); // Disable acq on next cycle, after this one has finished.

  // Wait for acq done IRQ.
  wait_for_exti();

  /*acq_init(0, svid, code_phase, carrier_freq); */
  acq_disable(); // Write to clear IRQ.
  acq_read_corr(corrs);
}

void do_acq(u8 svid, u16 cp_min, u16 cp_max, s16 cf_min, s16 cf_max, u16* cp, s16* cf, float* snr) {

  corr_t cs[ACQ_N_TAPS];

  float mag, mag_sq, best_mag = 0;
  float sd, mean = 0, sq_mean = 0;
  u32 count = 0;
  u16 best_cp;
  s16 best_cf;

  for (s16 carrier_freq = cf_min; carrier_freq <= cf_max; carrier_freq += 20) {
    for (u16 code_phase = cp_min; code_phase <= cp_max; code_phase += ACQ_N_TAPS) {
      do_one_acq(svid, code_phase, carrier_freq, cs); 
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
      }
      count += ACQ_N_TAPS;
    }
  }

  sd = sqrt(count*sq_mean - mean*mean) / count;
  mean = mean / count;

  *cp = best_cp;
  *cf = best_cf;
  *snr = (best_mag - mean) / sd;

}

void track_init(u8 channel, u8 svid, s32 starting_carrier_phase, u16 starting_code_phase) {
  //for length(svid)=5,length(starting_carrier_phase)=24,
  //  length(starting_code_phase) = 14
  u8 temp[6] = {0, 0, 0, 0, 0, 0};

  temp[0] = (svid & 0x1F) | (starting_carrier_phase << 5 & 0xE0);
  temp[1] = (starting_carrier_phase << 5) >> 8;
  temp[2] = (starting_carrier_phase << 5) >> 16;
  temp[3] = (((starting_carrier_phase << 5) >> 24) & 0x1F) | (starting_code_phase << 5);
  temp[4] = (starting_code_phase << 5) >> 8;
  temp[5] = ((starting_code_phase << 5) >> 16) & 0x07;

  swift_nap_xfer(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_INIT_OFFSET, 6, 0, temp);
}

void track_update(u8 channel, s32 carrier_freq, u32 code_phase_rate) {
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
  u8 temp[2*3 * 3]; // 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8)
  
  swift_nap_xfer(SPI_ID_TRACK_BASE + channel*TRACK_SIZE + TRACK_CORR_OFFSET, 2*3*3, temp, temp);

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

