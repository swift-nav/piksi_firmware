/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_gps_l2c.h"
#include "decode.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h> /* For BIT_POLARITY_... constants */
#include <libswiftnav/cnav_msg.h>
#include <assert.h>

#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"

typedef struct {
  cnav_msg_t cnav_msg;
  cnav_msg_decoder_t cnav_msg_decoder;
} gps_l2c_decoder_data_t;

static decoder_t gps_l2c_decoders[NUM_GPS_L2CM_DECODERS];
static gps_l2c_decoder_data_t gps_l2c_decoder_data[NUM_GPS_L2CM_DECODERS];

static void decoder_gps_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);
static void decoder_gps_l2c_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);
static void decoder_gps_l2c_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l2c = {
  .code =         CODE_GPS_L2CM,
  .init =         decoder_gps_l2c_init,
  .disable =      decoder_gps_l2c_disable,
  .process =      decoder_gps_l2c_process,
  .decoders =     gps_l2c_decoders,
  .num_decoders = NUM_GPS_L2CM_DECODERS
};

static decoder_interface_list_element_t list_element_gps_l2c = {
  .interface = &decoder_interface_gps_l2c,
  .next = 0
};

void decode_gps_l2c_register(void)
{
    for (u32 i=0; i<NUM_GPS_L2CM_DECODERS; i++) {
    gps_l2c_decoders[i].active = false;
    gps_l2c_decoders[i].data = &gps_l2c_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l2c);
}

static void decoder_gps_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data)
{
  (void)channel_info;
  gps_l2c_decoder_data_t *data = decoder_data;
  cnav_msg_decoder_init(&data->cnav_msg_decoder);
}

static void decoder_gps_l2c_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l2c_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data)
{
  gps_l2c_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit)) {
    /* Update TOW */
    u8 symbol_probability;
    u32 delay;
    s32 tow_ms;

    /* Symbol value probability, where 0x00 - 100% of 0, 0xFF - 100% of 1. */
    symbol_probability = soft_bit + 128;

    bool decoded = cnav_msg_decoder_add_symbol(&data->cnav_msg_decoder,
                                              symbol_probability,
                                              &data->cnav_msg,
                                              &delay);

    if (!decoded) {
      continue;
    }

    tow_ms = data->cnav_msg.tow * GPS_CNAV_MSG_LENGTH * GPS_L2C_SYMBOL_LENGTH;
    tow_ms += delay * GPS_L2C_SYMBOL_LENGTH;

    s8 bit_polarity = data->cnav_msg.bit_polarity;

    if ((tow_ms >= 0) && (bit_polarity != BIT_POLARITY_UNKNOWN)) {
      if (!tracking_channel_time_sync(channel_info->tracking_channel, tow_ms,
                                      bit_polarity)) {
        log_warn_sid(channel_info->sid, "TOW set failed");
      }
    }
  }
}
