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
 * These messages are in the implementation defined range (`0x0000-0x00FF`).
 * \{ */

#define MSG_PRINT                   0x10  /**< Piksi  -> Host  */
#define MSG_DEBUG_VAR               0x11  /**< Piksi  -> Host  */

#define MSG_ALMANAC                 0x69  /**< Host   -> Piksi */
#define MSG_SET_TIME                0x68  /**< Host   -> Piksi */

#define MSG_BOOTLOADER_HANDSHAKE    0xB0  /**< Host  <-> Piksi */
#define MSG_BOOTLOADER_JUMP_TO_APP  0xB1  /**< Host   -> Piksi */

#define MSG_RESET                   0xB2  /**< Host   -> Piksi */

#define MSG_CW_RESULTS              0xC0  /**< Piksi  -> Host  */
#define MSG_CW_START                0xC1  /**< Host   -> Piksi */

#define MSG_NAP_DEVICE_DNA          0xDD  /**< Host  <-> Piksi */

#define MSG_FLASH_PROGRAM           0xE0  /**< Host   -> Piksi */
#define MSG_FLASH_DONE              0xE0  /**< Piksi  -> Host  */
#define MSG_FLASH_READ              0xE1  /**< Host  <-> Piksi */
#define MSG_FLASH_ERASE             0xE2  /**< Host   -> Piksi */

#define MSG_STM_FLASH_LOCK_SECTOR   0xE3  /**< Host   -> Piksi */
#define MSG_STM_FLASH_UNLOCK_SECTOR 0xE4  /**< Host   -> Piksi */

#define MSG_STM_UNIQUE_ID           0xE5  /**< Host  <-> Piksi */

#define MSG_M25_FLASH_WRITE_STATUS  0xF3  /**< Host   -> Piksi */

#define MSG_RESET_FILTERS           0x22  /**< Host   -> Piksi */
#define MSG_INIT_BASE               0x23  /**< Host   -> Piksi */

#define MSG_SETTINGS                0xA0  /**< Host  <-> Piksi */
#define MSG_SETTINGS_SAVE           0xA1  /**< Host   -> Piksi */
#define MSG_SETTINGS_READ_BY_INDEX  0xA2  /**< Host   -> Piksi */

#define MSG_FILEIO_READ             0xA8  /**< Host  <-> Piksi */
#define MSG_FILEIO_READ_DIR         0xA9  /**< Host  <-> Piksi */
#define MSG_FILEIO_REMOVE           0xAC  /**< Host   -> Piksi */
#define MSG_FILEIO_WRITE            0xAD  /**< Host  <-> Piksi */

#define MSG_OBS_HDR                 0x40  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  gps_time_t t; /**< GPS time of observation. */
  u8 count;     /**< Serial count of obervation. */
  u8 n_obs;     /**< Number of observation records to follow. */
} msg_obs_hdr_t;

#define MSG_OBS                     0x41  /**< Piksi  -> Host  */
#define MSG_OLD_OBS                 0x42  /**< Piksi  -> Host  */
#define MSG_PACKED_OBS              0x45  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  struct __attribute__((packed)) {
    u32 tow;  /**< milliseconds since start of week. */
    u16 wn;   /**< GPS Week Numer. */
  } t;        /**< Compcated millisecond-accurate GPS time. */
  u8 seq;     /**< First nibble is the size of the sequence (n), second
                   nibble is the zero-indexed counter (ith packet of n) */
} msg_obs_header_t;

#define MSG_BASE_POS                0x44
typedef struct __attribute__((packed)) {
  double pos_llh[3];
} msg_base_pos_t;

typedef struct __attribute__((packed)) {
  u32 P;     /**< Pseudorange (cm) */
  struct __attribute__((packed)) carrier {
    s32 Li;  /**< Carrier phase (integer seconds) */
    u8 Lf;   /**< Carrier phase (scaled fractional seconds) */
  } L;       /**< Fixed point carrier phase (seconds) */
  u8 snr;    /**< Signal-to-Noise ratio (cn0 * 4 for 0.25 precision and
                  0-64 range) */
  u16 lock_counter; /**< Lock counter. Increments on new lock. */
  u8 prn;    /**< Satellite number. */
} msg_obs_content_t;

#define MSG_TRACKING_STATE        0x16  /**< Piksi  -> Host  */
#define MSG_IAR_STATE             0x19  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  u32 num_hyps;
} msg_iar_state_t;

#define MSG_THREAD_STATE          0x17  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  char name[20];
  u16 cpu;
  u32 stack_free;
} msg_thread_state_t;

#define MSG_UART_STATE            0x18  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  s32 avg;
  s32 min;
  s32 max;
  s32 current;
} latency_t;

typedef struct __attribute__((packed)) {
  struct __attribute__((packed)) {
    float tx_throughput;
    float rx_throughput;
    u16 crc_error_count;
    u16 io_error_count;
    u8 tx_buffer_level;
    u8 rx_buffer_level;
  } uarts[3];
  latency_t obs_latency;
} msg_uart_state_t;

#define MSG_EPHEMERIS             0x1A  /**< Piksi  -> Host  */

#define MSG_ACQ_RESULT            0x15  /**< Piksi  -> Host  */
typedef struct __attribute__((packed)) {
  float snr; /* SNR of best point. */
  float cp;  /* Code phase of best point. */
  float cf;  /* Carr freq of best point. */
  u8 prn;    /* PRN searched for. */
} acq_result_msg_t;

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK
#define MSG_OBS_TOW_MULTIPLIER ((double)1000.0)

#define MSG_OBS_P_MULTIPLIER ((double)1e2)
#define MSG_OBS_SNR_MULTIPLIER ((float)4)
#define MSG_OSB_LF_MULTIPLIER ((double)(1<<8))

void unpack_obs_header(msg_obs_header_t *msg,
  gps_time_t* t, u8* total, u8* count);

void pack_obs_header(gps_time_t *t, u8 total, u8 count,
  msg_obs_header_t *msg);

void unpack_obs_content(msg_obs_content_t *msg,
  double *P, double *L, double *snr, u16 *lock_counter, u8 *prn);

void pack_obs_content(double P, double L, double snr, u16 lock_counter, u8 prn,
  msg_obs_content_t *msg);

/** Value specifying the size of the SBP framing */
#define SBP_FRAMING_SIZE_BYTES 8
/** Value defining maximum SBP packet size */
#define SBP_FRAMING_MAX_PAYLOAD_SIZE 255

/** \} */

/** \} */

#endif  /* SWIFTNAV_SBP_PIKSI_H */

