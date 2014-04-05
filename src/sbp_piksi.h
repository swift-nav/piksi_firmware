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

#ifndef SWIFTNAV_SBP_PIKSI_H
#define SWIFTNAV_SBP_PIKSI_H

#include <libswiftnav/gpstime.h>

/** \addtogroup sbp
 * \{ */

/** \defgroup msgs Piksi Message Types
 * Swift Binary Protocol Message Types for Piksi defined messages.
 *
 * These messages are in the implimentation defined range (`0x0000-0x00FF`).
 * \{ */

#define MSG_PRINT                   0x10  /**< Piksi  -> Host  */
#define MSG_DEBUG_VAR               0x11  /**< Piksi  -> Host  */

#define MSG_ALMANAC                 0x69  /**< Host   -> Piksi */
#define MSG_SET_TIME                0x68  /**< Host   -> Piksi */

#define MSG_BOOTLOADER_HANDSHAKE    0xB0  /**< Host  <-> Piksi */
#define MSG_BOOTLOADER_JUMP_TO_APP  0xB1  /**< Host   -> Piksi */

#define MSG_RESET                   0xB2  /**< Host   -> Piksi */

#define MSG_CW_START                0xC1  /**< Host   -> Piksi */
#define MSG_CW_RESULTS              0xC0  /**< Piksi  -> Host  */

#define MSG_NAP_DEVICE_DNA          0xDD  /**< Host  <-> Piksi */

#define MSG_FLASH_PROGRAM           0xE0  /**< Host   -> Piksi */
#define MSG_FLASH_DONE              0xE0  /**< Piksi  -> Host  */
#define MSG_FLASH_READ              0xE1  /**< Host  <-> Piksi */
#define MSG_FLASH_ERASE             0xE2  /**< Host   -> Piksi */

#define MSG_STM_UNIQUE_ID           0xE5  /**< Host  <-> Piksi */

#define MSG_STM_FLASH_LOCK_SECTOR   0xE3  /**< Host   -> Piksi */
#define MSG_STM_FLASH_UNLOCK_SECTOR 0xE4  /**< Host   -> Piksi */

#define MSG_M25_FLASH_WRITE_STATUS  0xF3  /**< Host   -> Piksi */

#define MSG_SOLUTION                0x20  /**< Piksi  -> Host  */
#define MSG_DOPS                    0x21  /**< Piksi  -> Host  */

#define MSG_SIMULATION_MODE         0x94  /**< Host  <-> Piksi */
#define MSG_SIMULATION_SETTINGS     0x93  /**< Host  <-> Piksi */

#define MSG_BASELINE                0x23  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  double ned[3]; /**< Baseline in local North, East, Down frame (m). */
  gps_time_t t;  /**< GPS time of baseline solution. */
  u16 flags;     /**< Baseline solution flags. TODO: Add defs. */
  u8 n_sats;     /**< Number of satellites used in solution. */
} msg_baseline_t;

#define MSG_OBS_HDR                 0x40  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  gps_time_t t; /**< GPS time of observation. */
  u8 count;     /**< Serial count of obervation. */
  u8 n_obs;     /**< Number of observation records to follow. */
} msg_obs_hdr_t;

#define MSG_OBS                     0x41  /**< Piksi  -> Host  */
#define MSG_NEW_OBS                 0x42  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  double P;      /**< Pseudorange (m) */
  double L;      /**< Carrier-phase (cycles) */
  //float D;       /**< Doppler frequency (Hz) */
  float snr;     /**< Signal-to-Noise ratio */
  //u8 lock_count; /**< Number of epochs that phase lock has been maintained. */
  //u8 signal;     /**< Upper nibble: Satellite system designator,
  //                    Lower nibble: Signal type designator.
  //                    TODO: Add defs.*/
  u8 prn;        /**< Satellite number. */
  //u8 flags;      /**< Observation flags. TODO: Add defs. */
  //u8 obs_n;      /**< Observation number in set. */
} msg_obs_t;

#define MSG_TRACKING_STATE        0x16  /**< Piksi  -> Host  */

#define MSG_HEARTBEAT             0x01  /**< Piksi  -> Host  */

#define MSG_THREAD_STATE          0x17  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  char name[20];
  u16 cpu;
} msg_thread_state_t;

#define MSG_UART_STATE            0x18  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  struct __attribute__((packed)) {
    u16 crc_error_count;
    u8 tx_buffer_level;
    u8 rx_buffer_level;
  } uarts[3];
} msg_uart_state_t;

/** \} */

/** \} */

#endif  /* SWIFTNAV_SBP_PIKSI_H */

