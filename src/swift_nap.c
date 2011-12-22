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

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/timer.h>

#include "swift_nap.h"
#include "board/spi.h"

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
    for (u8 i=0; i<n_bytes; i++)
      data_in[i] = spi_xfer(SPI_BUS_FPGA, data_out[i]);
  else
    for (u8 i=0; i<n_bytes; i++)
      spi_xfer(SPI_BUS_FPGA, data_out[i]);

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
  timer_set_prescaler(TIM2, rcc_ppre1_frequency / 16368000);

  /* Set time auto-reload value to a 20ms period. */
  timer_set_period(TIM2, 327360);
  /* Set time auto-reload value to the longest possible period. */
  /*timer_set_period(TIM2, 0xFFFFFFFF);*/

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

u32 acq_init(u8 svid, u16 code_phase, s16 carrier_freq) {
  u32 temp = 0;

  temp |= svid & 0x1F;
  temp |= ((u32)code_phase << 5) & 0x1FFE0;
  temp |= ((u32)carrier_freq << 17) & 0x1FFE0000;
  temp |= (1 << 29); /* Acq enabled */

  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);

  return temp;
}

void acq_disable() {
  u32 temp = 0;
  swift_nap_xfer(SPI_ID_ACQ_INIT, 4, 0, (u8*)&temp);
}
