/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 * Copyright (C) 2012 Colin Beighley <colin@swift-nav.com>
 *
 */

#include <stdio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "acq.h"
#include "hw/spi.h"
#include "hw/leds.h"
#include "hw/m25_flash.h"
#include "hw/stm_flash.h"

int main(void)
{

  led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Setup and hold the FPGA PROGRAM_B line low so that the FPGA does not
   * contest the flash SPI bus */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_clear(GPIOC, GPIO12);

  spi_setup();
  debug_setup(1);
  m25_setup();
  stm_flash_callbacks_setup();

  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- M25 FLASH TEST ---\n");

  while (1) {
    for (u32 i = 0; i < 10000; i++)
      __asm__("nop");
    DO_EVERY(20,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
    debug_process_messages();
  }

  return 0;
}
