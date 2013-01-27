/*
 * This file is part of the Paparazzi UAV project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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

#include <string.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/scb.h>
#include <libopencm3/stm32/f4/usart.h>

#include "main.h"
#include "hw/leds.h"

#define APP_ADDRESS	0x08002000

/* Commands sent with wBlockNum == 0 as per ST implementation. */
//#define CMD_SETADDR	0x21 
//#define CMD_ERASE	0x41 

void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);

	/* Enable clocks for USART6. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART6EN);
}

void usart_setup(void)
{
  /* Setup USART6 pins */
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
	gpio_set_af(GPIOC, GPIO_AF8, GPIO6|GPIO7);

	/* Setup USART6 parameters. */
	usart_set_baudrate(USART6, 230400);
	usart_set_databits(USART6, 8);
	usart_set_stopbits(USART6, USART_STOPBITS_1);
	usart_set_mode(USART6, USART_MODE_TX_RX);
	usart_set_parity(USART6, USART_PARITY_NONE);
	usart_set_flow_control(USART6, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART6);
}

void print_mem(u32 address){
  u8 foo = *(u8 *)address;
  if (((foo >> 4) & 0x0F) < 10) { //0-9
    usart_send_blocking(USART6, ((foo >> 4) & 0x0F) + 48);
  } else { //A-F
    usart_send_blocking(USART6, ((foo >> 4) & 0x0F) + 55);
  }
  if ((foo & 0x0F) < 10) { //0-9
    usart_send_blocking(USART6, (foo & 0x0F) + 48);
  } else { //A-F
    usart_send_blocking(USART6, (foo & 0x0F) + 55);
  }
  usart_send_blocking(USART6, '\n');
  usart_send_blocking(USART6, '\r');
}

int main(void)
{
  /* Setup clock for UART and LEDs */
  clock_setup();
  /* Setup UART6 to receive data from flash */
  usart_setup();
  /* Setup and turn on LEDs */
  led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Boot the application if it's valid */
  if ((*(volatile u32*)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
    /* Set vector table base address */
    SCB_VTOR = APP_ADDRESS & 0xFFFF;
    /* Initialise master stack pointer */
    asm volatile ("msr msp, %0"::"g"
        (*(volatile u32*)APP_ADDRESS));
    /* Jump to application */
    (*(void(**)())(APP_ADDRESS + 4))();
  }

  usart_send_blocking(USART6, '\n');
  usart_send_blocking(USART6, '\r');

  //Try some flash writing stuff
  //Bootloader starts at 0x08000000 and shouldn't go above 0x08003FFF
  //So let's write some stuff to addresses starting at 0x08004000
  flash_unlock();
  print_mem(0x08004000);
//  flash_erase_sector(*(u32*)(prog.buf+1));
  flash_erase_sector(0x0000 | (0x1 << 3),1);
  print_mem(0x08004000);
  flash_program_byte(0x08004000,0xA5,0x0020);
  print_mem(0x08004000);
  flash_lock();

  u16 uart_data;
	
	while (1){
    /* Echo if we have a character ready on the UART */
    if (USART_SR(USART6) & USART_SR_RXNE) {
      uart_data = usart_recv(USART6);
      usart_send_blocking(USART6, uart_data);
      usart_send_blocking(USART6, uart_data);
    }
    DO_EVERY(500000,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
  }
}
