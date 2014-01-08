/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
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
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/usart.h>

#include <libswiftnav/edc.h>

#include "board/leds.h"
#include "peripherals/usart.h"
#include "error.h"
#include "sbp.h"
#include "settings.h"

/** \addtogroup io
 * \{ */

/** \addtogroup error
 * Last resort, low-level, blocking, continuous error messages.
 * \{ */

/** Error message.
 * Halts the program while continually sending a fixed error message in SBP
 * message format to the FTDI USART, in a way that should get the message
 * through to the Python console even if it's interrupting another transmission.
 *
 * \param msg A pointer to an array of chars containing the error message.
 */
void screaming_death(char *msg)
{
  __asm__("CPSID if;");           /* Disable all interrupts and faults */

  DMA2_S7CR = 0;                  /* Disable USART6 TX DMA */
  USART6_CR3 &= ~USART_CR3_DMAT;  /* Disable USART6 DMA */

  /* If USART6 has not yet been set up, set it up */
  if (!(USART6_CR1 & USART_CR1_UE)) {
    RCC_APB2ENR |= RCC_APB2ENR_USART6EN; /* Enable clock to USART6 peripheral */
    RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;   /* Enable clock to USART6 pins */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, /* Set up USART6 pins */
                    GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOC, GPIO_AF8, GPIO6 | GPIO7);
    if (settings.settings_valid == VALID) {
      usart_set_parameters(USART6, settings.ftdi_usart.baud_rate);
    } else {
      usart_set_parameters(USART6, USART_DEFAULT_BAUD);
    }
  }

  #define MAX_MSG_N 64       /* Maximum length of error message */

  static char err_msg[MAX_MSG_N+6] = {SBP_HEADER_1, SBP_HEADER_2,
                                           MSG_PRINT, MAX_MSG_N,
                                           'E', 'R', 'R', 'O', 'R', ':', ' ',
                                           [11 ... MAX_MSG_N+2] = '!',
                                           [MAX_MSG_N+3] = '\n',
                                           /* CRC calculated below */
                                           [MAX_MSG_N+4] = 0x00,
                                           [MAX_MSG_N+5] = 0x00};

  /* Insert message */
  u8 i = 0;
  while (*msg && i < MAX_MSG_N)  /* Don't want to use C library memcpy */
    err_msg[11 + (i++)] = *msg++;

  /* Insert CRC */
  u16 crc = crc16_ccitt((u8*)&err_msg[2], 2, 0);
  crc = crc16_ccitt((u8*)&err_msg[4], MAX_MSG_N, crc);
  err_msg[MAX_MSG_N + 4] = crc & 0xFF;
  err_msg[MAX_MSG_N + 5] = (crc >> 8) & 0xFF;

  /* Continuously send error message */
  i = 0;
  while (1) {
    while (!(USART6_SR & USART_SR_TXE)) ;
    USART6_DR = err_msg[i];
    if (++i == (MAX_MSG_N + 6)) {
      i = 0;
      led_toggle(LED_RED);
      for (u32 d = 0; d < 5000000; d++)
        __asm__("nop");
    }
  }
}

/** \} */

/** \} */

