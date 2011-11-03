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

#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/nvic.h>

#include "../swift_nap.h"
#include "exti.h"
#include "leds.h"

#define DLL_IGAIN 1.431702e-2
#define DLL_PGAIN 5.297297
#define PLL_IGAIN 1.779535e+1
#define PLL_PGAIN 3.025210e+2

#define SAMP_FREQ 16.368e6

u32 exti_count = 0;
u32 data[10][100];

void exti_setup()
{
  // Signal from the FPGA is on PA0.
  
  RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable clock to GPIOA
  RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable clock to SYSCFG "peripheral", which we think contains the EXTI functionality.

  exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
  exti_reset_request(EXTI0);
	exti_enable_request(EXTI0);

	/* Enable EXTI0 interrupt */
	nvic_enable_irq(NVIC_EXTI0_IRQ);

}

s32 sign_extend(u32 n, u8 bits)
{
  if (n & (1 << (bits-1)))
    return ((s32)(n << (32-bits)) >> (32-bits));
  
  return (s32)n;
}

/*double calc_tau1(double lbw, double zeta, double k)*/
/*{*/
  /*double wn, tau1;*/

  /*wn = (lbw*8*zeta) / (4*zeta*zeta + 1);*/
  /*tau1 = k / (wn*wn);*/

  /*return tau1;*/
/*}*/

/*double calc_tau2(double lbw, double zeta, double k)*/
/*{*/
  /*double wn, tau2;*/

  /*wn = (lbw*8*zeta) / (4*zeta*zeta + 1);*/
  /*tau2 = (2*zeta)/wn;*/

  /*return tau1;*/
/*}*/


void exti0_isr()
{
  static double dll_disc = 0;
  static double pll_disc = 0;
  static double dll_disc_old;
  static double pll_disc_old;

  static double dll_freq = 1.023e6;
  static double pll_freq = -550.0;

  s32 dll_freq_fp, pll_freq_fp;

  double CIE;
  double CQE;
  double CIP;
  double CQP;
  double CIL;
  double CQL;

  exti_reset_request(EXTI0);
  led_on(LED_GREEN);
  gpio_set(GPIOC, GPIO11);

  CIE = sign_extend(swift_nap_read(0, 17), 22);
  CQE = sign_extend(swift_nap_read(0, 18), 22);
  CIP = sign_extend(swift_nap_read(0, 19), 22);
  CQP = sign_extend(swift_nap_read(0, 20), 22);
  CIL = sign_extend(swift_nap_read(0, 21), 22);
  CQL = sign_extend(swift_nap_read(0, 22), 22);

  dll_disc_old = dll_disc;
  pll_disc_old = pll_disc;

  // TODO: check for divide by zero
  pll_disc = atan(CQP/CIP)/(2*3.14159);

  pll_freq = pll_freq + PLL_PGAIN*(pll_disc-pll_disc_old) \
             + PLL_IGAIN*pll_disc;


  dll_disc = (sqrt(CIE*CIE + CQE*CQE) - sqrt(CIL*CIL + CQL*CQL)) \
             / (sqrt(CIE*CIE + CQE*CQE) + sqrt(CIL*CIL + CQL*CQL));

  dll_freq = dll_freq + DLL_PGAIN*(dll_disc-dll_disc_old) \
             + DLL_IGAIN*dll_disc;

  pll_freq_fp = (s32)(pll_freq*pow(2,24)/SAMP_FREQ);
  dll_freq_fp = (s32)(dll_freq*pow(2,32)/SAMP_FREQ);

  swift_nap_write(0, 2, pll_freq_fp);
  swift_nap_write(0, 12, dll_freq_fp);

  /*swift_nap_write(0,2,pll_freq_sim[exti_count]);*/
  /*swift_nap_write(0,12,dll_freq_sim[exti_count]);*/
  
  //swift_nap_write(0,2,0x0000fdcc);//carr_pr_s32);
  //swift_nap_write(0,12,0x10000000);// code_pr_s32);

  if (exti_count < 100) {
    data[0][exti_count] = (s32)CIE;
    data[1][exti_count] = (s32)CQE;
    data[2][exti_count] = (s32)CIP;
    data[3][exti_count] = (s32)CQP;
    data[4][exti_count] = (s32)CIL;
    data[5][exti_count] = (s32)CQL;
    data[6][exti_count] = (s32)pll_freq;
    data[7][exti_count] = (s32)dll_freq;
    data[8][exti_count] = (s32)(pll_disc*1e6);
    data[9][exti_count] = (s32)(dll_disc*1e6);
    exti_count++;
  }  

  led_off(LED_GREEN);
  gpio_clear(GPIOC, GPIO11);
}
