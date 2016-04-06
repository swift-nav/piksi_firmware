/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *          Gareth McMullin <gareth@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <hal.h>

#include "frontend.h"

#define FRONTEND_SPI SPID2

const SPIConfig spi_config = {0, SPI_MODE_0,
                              SPI_CLK_DIV_16, SPI_SS_GPIO_LINE};

static u8 spi_write(u8 reg, u8 data)
{
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];

  spiExchange(&FRONTEND_SPI, sizeof(send_buf), send_buf, recv_buf);

  return recv_buf[1];
}

void frontend_configure(void)
{
  spiStart(&FRONTEND_SPI, &spi_config);
  spiAcquireBus(&FRONTEND_SPI);
  spiSelect(&FRONTEND_SPI);

  for (u8 i = 0; i < 2; ++i) {
    spi_write(2, 0x03);
    spi_write(3, 0x01);
    spi_write(4, 0x03);
    spi_write(5, 0x00);
    spi_write(6, 0x1F);
    spi_write(9, 0x00);
    spi_write(11, 0x08);
    spi_write(12, 0x1C);
    spi_write(13, 0x03);
    spi_write(14, 0x69);
    spi_write(15, 0x2B);
    spi_write(16, 0x74);
    spi_write(17, 0xF1);
    spi_write(18, 0xEA);
    spi_write(19, 0x0B);
    spi_write(20, 0x01);
    spi_write(21, 0x28);
    spi_write(22, 0x2B);
    spi_write(23, 0x74);
    spi_write(24, 0xF1);
    spi_write(25, 0xEA);
    spi_write(26, 0x0B);
    spi_write(27, 0x01);
    spi_write(28, 0x28);
    spi_write(29, 0x2B);
    spi_write(30, 0x74);
    spi_write(31, 0xF1);
    spi_write(32, 0xEA);
    spi_write(33, 0x0B);
    spi_write(34, 0x03);
    spi_write(35, 0x7F);
    spi_write(36, 0x2B);
    spi_write(37, 0x74);
    spi_write(38, 0xF1);
    spi_write(39, 0xEA);
    spi_write(40, 0x0B);
    spi_write(41, 0x03);
    spi_write(42, 0x4F);
    spi_write(43, 0x89);
    spi_write(45, 0x01);
    spi_write(46, 0x7B);
    spi_write(47, 0x91);
    spi_write(49, 0xBD);
    spi_write(50, 0x0C);
    spi_write(51, 0x64);
    spi_write(52, 0xA8);
    spi_write(53, 0x5B);
    spi_write(54, 0xE5);
    spi_write(55, 0x57);
    spi_write(56, 0xE4);
    spi_write(57, 0xDD);
    spi_write(58, 0x0C);
    spi_write(59, 0x64);
    spi_write(60, 0xA8);
    spi_write(61, 0xC5);
    spi_write(62, 0x6D);
    spi_write(63, 0x57);
    spi_write(64, 0x71);
    spi_write(65, 0x7F);
    spi_write(66, 0x00);
    spi_write(67, 0xC9);
    spi_write(68, 0x86);
    spi_write(69, 0x7E);
    spi_write(70, 0x0F);
    spi_write(71, 0x11);
    spi_write(72, 0x00);
    spi_write(73, 0x00);
    spi_write(74, 0xFF);
    spi_write(75, 0x1C);
    spi_write(76, 0xCD);
    spi_write(77, 0x99);
    spi_write(78, 0x0B);
    spi_write(79, 0x40);
    spi_write(80, 0x08);
    spi_write(81, 0x30);
    spi_write(82, 0xFF);
    spi_write(83, 0x1C);
    spi_write(84, 0xCD);
    spi_write(85, 0x99);
    spi_write(86, 0x0B);
    spi_write(87, 0x40);
    spi_write(88, 0x08);
    spi_write(89, 0x30);
    spi_write(90, 0xFF);
    spi_write(91, 0x0C);
    spi_write(92, 0xCD);
    spi_write(93, 0x99);
    spi_write(94, 0x0B);
    spi_write(95, 0x40);
    spi_write(96, 0x08);
    spi_write(97, 0x30);
    spi_write(98, 0xFF);
    spi_write(99, 0x0C);
    spi_write(100, 0xCD);
    spi_write(101, 0x99);
    spi_write(102, 0x0B);
    spi_write(103, 0x40);
    spi_write(104, 0x08);
    spi_write(105, 0x30);
  }

  spi_write(15, 0x0B);
  spi_write(22, 0x0B);
  spi_write(29, 0x0B);
  spi_write(36, 0x0B);

  spiUnselect(&FRONTEND_SPI);
  spiReleaseBus(&FRONTEND_SPI);
}

void frontend_setup(void)
{
  /* Register any setting... */

  frontend_configure();
}

bool frontend_ant_status(void)
{
  return true;
}

antenna_type_t frontend_ant_setting(void)
{
  return EXTERNAL;
}

