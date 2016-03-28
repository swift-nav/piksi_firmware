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

#ifndef _BOARD_H_
#define _BOARD_H_

#define SD_FTDI  (&SD2)
#define SD_UARTA (&SD1)
#define SD_UARTB NULL

/*
 * Setup for the Digilent uZed board.
 */

/*
 * Board identifier.
 */
#define BOARD_DIGILENT_UZED
#define BOARD_NAME "Digilent uZed"

#define LINE_LED1 PAL_LINE(GPIO1, 15)
#define LINE_LED2 PAL_LINE(GPIO1, 19)

#define SPI_SS_GPIO_LINE PAL_LINE(GPIO0, 13)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void board_preinit_hook(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
