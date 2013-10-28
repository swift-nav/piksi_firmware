/*
 * Copyright (C) 2012-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_MESSAGES_H
#define SWIFTNAV_SBP_MESSAGES_H

/** \addtogroup sbp
 * \{ */

/** \defgroup msgs Message IDs
 * Swift Binary Protocol Message IDs.
 * \{ */

#define MSG_PRINT                   0x01  /**< Piksi  -> Host  */

#define MSG_TRACKING_STATE          0x22  /**< Piksi  -> Host  */

#define MSG_SOLUTION                0x50  /**< Piksi  -> Host  */
#define MSG_DOPS                    0x51  /**< Piksi  -> Host  */

#define MSG_ALMANAC                 0x69  /**< Host   -> Piksi */
#define MSG_SET_TIME                0x68  /**< Host   -> Piksi */

#define MSG_ACQ_RESULT              0xA0  /**< Piksi  -> Host  */

#define MSG_BOOTLOADER_HANDSHAKE    0xB0  /**< Host  <-> Piksi */
#define MSG_BOOTLOADER_JUMP_TO_APP  0xB1  /**< Host   -> Piksi */

#define MSG_RESET                   0xB2  /**< Host   -> Piksi */

#define MSG_CW_START                0xC1  /**< Host   -> Piksi */
#define MSG_CW_RESULTS              0xC0  /**< Piksi  -> Host  */

#define MSG_NAP_DEVICE_DNA          0xDD  /**< Host  <-> Piksi */

#define MSG_STM_FLASH_WRITE         0xE0  /**< Host   -> Piksi */
#define MSG_STM_FLASH_READ          0xE1  /**< Host  <-> Piksi */
#define MSG_STM_FLASH_ERASE         0xE2  /**< Host   -> Piksi */
#define MSG_STM_FLASH_DONE          0xE0  /**< Piksi  -> Host  */

#define MSG_STM_UNIQUE_ID           0xE5  /**< Host  <-> Piksi */

#define MSG_M25_FLASH_WRITE         0xF0  /**< Host   -> Piksi */
#define MSG_M25_FLASH_READ          0xF1  /**< Host  <-> Piksi */
#define MSG_M25_FLASH_ERASE         0xF2  /**< Host   -> Piksi */
#define MSG_M25_FLASH_DONE          0xF0  /**< Piksi  -> Host  */

/** \} */

/** \} */

#endif  /* SWIFTNAV_SBP_MESSAGES_H */

