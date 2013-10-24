/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_STM_FLASH_H
#define SWIFTNAV_STM_FLASH_H

/** \addtogroup stm_flash
 * \{ */

/* TODO : put this in libopencm3 */
#define STM_UNIQUE_ID_ADDR 0x1FFF7A10

/** \} */

void register_stm_flash_callbacks();
void stm_unique_id_callback_register(void);

#endif
