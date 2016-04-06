/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _MCUCONF_H_
#define _MCUCONF_H_

/*
 * ZYNQ7000 drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the driver
 * is enabled in halconf.h.
 */

#define ZYNQ7000_MCUCONF

#define ZYNQ7000_CPU_6x4x_FREQUENCY_Hz              666600000U
#define ZYNQ7000_CPU_3x2x_FREQUENCY_Hz              333300000U
#define ZYNQ7000_CPU_2x_FREQUENCY_Hz                222200000U
#define ZYNQ7000_CPU_1x_FREQUENCY_Hz                111100000U

/*
 * ST driver system settings.
 */
#define ZYNQ7000_ST_PRV_TIMER_IRQ_PRIORITY          30

/*
 * GPT driver system settings.
 */
#define ZYNQ7000_GPT_USE_TTC0_0                     TRUE
#define ZYNQ7000_GPT_USE_TTC0_1                     TRUE
#define ZYNQ7000_GPT_USE_TTC0_2                     FALSE
#define ZYNQ7000_GPT_USE_TTC1_0                     TRUE
#define ZYNQ7000_GPT_USE_TTC1_1                     TRUE
#define ZYNQ7000_GPT_USE_TTC1_2                     TRUE
#define ZYNQ7000_GPT_TTC0_0_IRQ_PRIORITY            4
#define ZYNQ7000_GPT_TTC0_1_IRQ_PRIORITY            4
#define ZYNQ7000_GPT_TTC0_2_IRQ_PRIORITY            4
#define ZYNQ7000_GPT_TTC1_0_IRQ_PRIORITY            4
#define ZYNQ7000_GPT_TTC1_1_IRQ_PRIORITY            4
#define ZYNQ7000_GPT_TTC1_2_IRQ_PRIORITY            4

/*
 * ADC driver system settings.
 */

/*
 * CAN driver system settings.
 */

/*
 * MAC driver system settings.
 */

/*
 * PWM driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define ZYNQ7000_SERIAL_UART_REFCLK_FREQUENCY_Hz    50000000U
#define ZYNQ7000_SERIAL_USE_UART0                   TRUE
#define ZYNQ7000_SERIAL_USE_UART1                   TRUE
#define ZYNQ7000_SERIAL_UART0_IRQ_PRIORITY          4
#define ZYNQ7000_SERIAL_UART1_IRQ_PRIORITY          4

/*
 * SPI driver system settings.
 */
#define ZYNQ7000_SPI_SPI_REFCLK_FREQUENCY_Hz        50000000U
#define ZYNQ7000_SPI_USE_SPI0                       TRUE
#define ZYNQ7000_SPI_USE_SPI1                       TRUE
#define ZYNQ7000_SPI_SPI0_IRQ_PRIORITY              4
#define ZYNQ7000_SPI_SPI1_IRQ_PRIORITY              4

/*
 * EXT driver system settings.
 */
#define ZYNQ7000_EXT_GPIO_IRQ_PRIORITY              4
#define ZYNQ7000_EXT_NUM_CHANNELS                   8

/*
 * WDG driver system settings.
 */
#define ZYNQ7000_WDG_USE_PRV_WDT                    TRUE

/*
 * AXI DMA driver system settings.
 */
#define ZYNQ7000_AXI_DMA_USE_AXI_DMA0               TRUE
#define ZYNQ7000_AXI_DMA0_MM2S_PRESENT              TRUE
#define ZYNQ7000_AXI_DMA0_S2MM_PRESENT              TRUE
#define ZYNQ7000_AXI_DMA0_MM2S_IRQ_PRIORITY         4
#define ZYNQ7000_AXI_DMA0_S2MM_IRQ_PRIORITY         4
#define ZYNQ7000_AXI_DMA_USE_AXI_DMA1               FALSE

#endif /* _MCUCONF_H_ */
