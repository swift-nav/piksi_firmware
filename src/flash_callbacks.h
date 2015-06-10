/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_FLASH_CALLBACKS_H
#define SWIFTNAV_FLASH_CALLBACKS_H

#define FLASH_STM 0 /**< Value to pass flash callbacks to use STM Flash */
#define FLASH_M25 1 /**< Value to pass flash callbacks to use M25 Flash */

#define FLASH_OK             0
#define FLASH_INVALID_FLASH  1
#define FLASH_INVALID_LEN    2
#define FLASH_INVALID_ADDR   3
#define FLASH_INVALID_RANGE  4
#define FLASH_INVALID_SECTOR 5

#define FLASH_ADDRS_PER_OP 128

void stm_unique_id_callback_register(void);

#endif /* SWIFTNAV_FLASH_CALLBACKS_H */
