/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libsbp/flash.h>

#include "sbp.h"
#include "peripherals/stm_flash.h"
#include "flash_callbacks.h"

/** Callback to read STM32F4's hardcoded unique ID.
 * Sends STM32F4 unique ID (12 bytes) back to host.
 */
void stm_unique_id_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  sbp_send_msg(SBP_MSG_STM_UNIQUE_ID, 12, (u8*)STM_UNIQUE_ID_ADDR);
}

/** Register callback to read Device's Unique ID. */
void stm_unique_id_callback_register(void)
{
  static sbp_msg_callbacks_node_t stm_unique_id_node;

  sbp_register_cbk(SBP_MSG_STM_UNIQUE_ID,
                        &stm_unique_id_callback,
                        &stm_unique_id_node);
}

