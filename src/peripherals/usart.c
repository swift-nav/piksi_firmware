/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/usart.h>

#include <ch.h>
#include <stdio.h>

#include "../settings.h"
#include "usart.h"
#include "3drradio.h"

/** \addtogroup io
 * \{ */


static const char const * portmode_enum[] = {"NONE", "SBP", "NMEA", "RTCM", NULL};
static struct setting_type portmode;

usart_settings_t ftdi_usart = {
  .mode               = SBP,
  .baud_rate          = USART_DEFAULT_BAUD_FTDI,
  .sbp_message_mask   = 0xFFFF,
  .configure_telemetry_radio_on_boot = 0,
};

usart_settings_t uarta_usart = {
  .mode               = SBP,
  .baud_rate          = USART_DEFAULT_BAUD_TTL,
  .sbp_message_mask   = 0x40,
  .configure_telemetry_radio_on_boot = 1,
};

usart_settings_t uartb_usart = {
  .mode             = SBP,
  .baud_rate        = USART_DEFAULT_BAUD_TTL,
  .sbp_message_mask = 0xFF00,
  .configure_telemetry_radio_on_boot = 1,
};

bool all_uarts_enabled = false;

/** \} */

/** \addtogroup peripherals
 * \{ */

/** \defgroup usart USART
 * Functions to setup and use STM32F4 USART peripherals with DMA.
 * \{ */

usart_rx_dma_state ftdi_rx_state;
usart_tx_dma_state ftdi_tx_state;
usart_rx_dma_state uarta_rx_state;
usart_tx_dma_state uarta_tx_state;
usart_rx_dma_state uartb_rx_state;
usart_tx_dma_state uartb_tx_state;

/** Set up USART parameters for particular USART.
 * \param usart USART to set up parameters for.
 * \param baud  Baud rate to set.
 */
void usart_set_parameters(u32 usart, u32 baud)
{
  /* Setup UART parameters. */
  baud = baud;
  usart_disable(usart);
  usart_set_baudrate(usart, baud);
  usart_set_databits(usart, 8);
  usart_set_stopbits(usart, USART_STOPBITS_1);
  usart_set_parity(usart, USART_PARITY_NONE);
  usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
  usart_set_mode(usart, USART_MODE_TX_RX);

  /* Enable the USART. */
  usart_enable(usart);
}

/** Set up the USART peripherals, hook them into the settings subsystem
*
*/
void usarts_setup()
{

  radio_setup();

  int TYPE_PORTMODE = settings_type_register_enum(portmode_enum, &portmode);

  SETTING("uart_ftdi", "mode", ftdi_usart.mode, TYPE_PORTMODE);
  SETTING("uart_ftdi", "sbp_message_mask", ftdi_usart.sbp_message_mask, TYPE_INT);
  SETTING_NOTIFY("uart_ftdi", "baudrate", ftdi_usart.baud_rate, TYPE_INT,
                 baudrate_change_notify);

  SETTING("uart_uarta", "mode", uarta_usart.mode, TYPE_PORTMODE);
  SETTING("uart_uarta", "sbp_message_mask", uarta_usart.sbp_message_mask, TYPE_INT);
  SETTING("uart_uarta", "configure_telemetry_radio_on_boot",
          uarta_usart.configure_telemetry_radio_on_boot, TYPE_BOOL);
  SETTING_NOTIFY("uart_uarta", "baudrate", uarta_usart.baud_rate, TYPE_INT,
          baudrate_change_notify);

  SETTING("uart_uartb", "mode", uartb_usart.mode, TYPE_PORTMODE);
  SETTING("uart_uartb", "sbp_message_mask", uartb_usart.sbp_message_mask, TYPE_INT);
  SETTING("uart_uartb", "configure_telemetry_radio_on_boot",
          uartb_usart.configure_telemetry_radio_on_boot, TYPE_BOOL);
  SETTING_NOTIFY("uart_uartb", "baudrate", uartb_usart.baud_rate, TYPE_INT,
          baudrate_change_notify);

  usarts_enable(ftdi_usart.baud_rate, uarta_usart.baud_rate, uartb_usart.baud_rate, true);

}

/** Callback for settings subsystem changing the baudrate of a UART.
*
*/
bool baudrate_change_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    usarts_disable();
    usarts_enable(ftdi_usart.baud_rate, uarta_usart.baud_rate, uartb_usart.baud_rate, false);
    return true;
  }
  return false;
}


/** Enable the USART peripherals.
 * USART 6, 1 and 3 peripherals are configured
 * (connected to the FTDI, UARTA and UARTB ports on the Piksi respectively).
 */
void usarts_enable(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud, bool do_preconfigure_hooks)
{

  /* Ensure that the first time around, we do the preconfigure hooks */
  if (!all_uarts_enabled && !do_preconfigure_hooks)
    return;

  /* First give everything a clock. */

  /* Clock the USARTs. */
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN;
  RCC_APB1ENR |= RCC_APB1ENR_USART3EN;

  /* GPIO pins corresponding to the USART. */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPCEN;

  /* Assign the GPIO pins appropriately:
   *
   * USART   TX    RX  Connection
   * ----------------------------
   * 6      PC6   PC7  FTDI
   * 1      PA9  PA10  UARTA
   * 3     PC10  PC11  UARTB
   */

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
  gpio_set_af(GPIOC, GPIO_AF8, GPIO6 | GPIO7);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10 | GPIO11);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO10 | GPIO11);

  usart_set_parameters(USART6, ftdi_baud);
  usart_set_parameters(USART1, uarta_baud);
  usart_set_parameters(USART3, uartb_baud);

  /* FTDI (USART6) TX - DMA2, stream 6, channel 5. */
  usart_tx_dma_setup(&ftdi_tx_state, USART6, DMA2, 6, 5);
  /* FTDI (USART6) RX - DMA2, stream 1, channel 5. */
  usart_rx_dma_setup(&ftdi_rx_state, USART6, DMA2, 1, 5);

  if (do_preconfigure_hooks) {

    printf("\n\nPiksi Starting...\n"
       "Firmware Version: " GIT_VERSION "\n" \
       "Built: " __DATE__ " " __TIME__ "\n");

    if (uarta_usart.configure_telemetry_radio_on_boot) {
      radio_preconfigure_hook(USART1, uarta_baud, "UARTA");
    }
    if (uartb_usart.configure_telemetry_radio_on_boot) {
      radio_preconfigure_hook(USART3, uartb_baud, "UARTB");
    }
  }

  /* UARTA (USART1) TX - DMA2, stream 7, channel 4. */
  usart_tx_dma_setup(&uarta_tx_state, USART1, DMA2, 7, 4);
  /* UARTA (USART1) RX - DMA2, stream 2, channel 4. */
  usart_rx_dma_setup(&uarta_rx_state, USART1, DMA2, 2, 4);

  /* UARTB (USART3) TX - DMA1, stream 3, channel 4. */
  usart_tx_dma_setup(&uartb_tx_state, USART3, DMA1, 3, 4);
  /* UARTB (USART3) RX - DMA1, stream 1, channel 4. */
  usart_rx_dma_setup(&uartb_rx_state, USART3, DMA1, 1, 4);

  all_uarts_enabled = true;

}

/** Disable all USARTs. */
void usarts_disable()
{
  /* Disable DMA channels. */
  /* Disable all USARTs. */

  if (!all_uarts_enabled)
    return;

  usart_tx_dma_disable(&ftdi_tx_state);
  usart_rx_dma_disable(&ftdi_rx_state);
  usart_disable(USART6);

  usart_tx_dma_disable(&uarta_tx_state);
  usart_rx_dma_disable(&uarta_rx_state);
  usart_disable(USART1);

  usart_tx_dma_disable(&uartb_tx_state);
  usart_rx_dma_disable(&uartb_rx_state);
  usart_disable(USART3);
}

/** DMA 2 Stream 6 Interrupt Service Routine. */
void dma2_stream6_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_tx_dma_isr(&ftdi_tx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}
/** DMA 2 Stream 1 Interrupt Service Routine. */
void dma2_stream1_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_rx_dma_isr(&ftdi_rx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}
/** DMA 2 Stream 7 Interrupt Service Routine. */
void dma2_stream7_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_tx_dma_isr(&uarta_tx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}
/** DMA 2 Stream 2 Interrupt Service Routine. */
void dma2_stream2_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_rx_dma_isr(&uarta_rx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}
/** DMA 1 Stream 3 Interrupt Service Routine. */
void dma1_stream3_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_tx_dma_isr(&uartb_tx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}
/** DMA 1 Stream 1 Interrupt Service Routine. */
void dma1_stream1_isr(void)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  usart_rx_dma_isr(&uartb_rx_state);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

/** \} */

/** \} */

