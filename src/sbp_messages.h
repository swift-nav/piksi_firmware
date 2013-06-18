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

#define MSG_PRINT 0x01 /**< Callback in Python */

#define MSG_ALMANAC   0x69 /**< Callback in C */
#define MSG_SET_TIME  0x68 /**< Callback in C */

#define MSG_BOOTLOADER_HANDSHAKE   0xB0 /**< Callback in C and Python */
#define MSG_BOOTLOADER_JUMP_TO_APP 0xB1 /**< Callback in C */

#define MSG_CW_START   0xC1 /**< Callback in C */
#define MSG_CW_RESULTS 0xC0 /**< Callback in Python */

#define MSG_NAP_DEVICE_DNA 0xDD /**< Callback in C and Python */

#define MSG_STM_FLASH_WRITE 0xE0 /**< Callback in C */
#define MSG_STM_FLASH_READ  0xE1 /**< Callback in C and Python */
#define MSG_STM_FLASH_ERASE 0xE2 /**< Callback in C */
#define MSG_STM_FLASH_DONE  0xE0 /**< Callback in Python */

#define MSG_STM_UNIQUE_ID 0xE5 /**< Callback in C and Python */

#define MSG_M25_FLASH_WRITE 0xF0 /**< Callback in C */
#define MSG_M25_FLASH_READ  0xF1 /**< Callback in C and Python */
#define MSG_M25_FLASH_ERASE 0xF2 /**< Callback in C */
#define MSG_M25_FLASH_DONE  0xF0 /**< Callback in Python */

#define MSG_SOLUTION 0x50 /**< Callback in Python */
#define MSG_DOPS     0x51 /**< Callback in Python */
#define MSG_PR_ERRS  0x52 /**< Callback in Python */

#define MSG_TRACKING_STATE 0x22 /**< Callback in Python */

/** \} */

/** \} */

#endif /* SWIFTNAV_SBP_MESSAGES_H */
