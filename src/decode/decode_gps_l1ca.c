/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_gps_l1ca.h"
#include "decode.h"

#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h>
#include <assert.h>

#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"
#include "l2c_capb.h"
#include "iono.h"

typedef struct {
  nav_msg_t nav_msg;
} gps_l1ca_decoder_data_t;

static decoder_t gps_l1ca_decoders[NUM_GPS_L1CA_DECODERS];
static gps_l1ca_decoder_data_t gps_l1ca_decoder_data[NUM_GPS_L1CA_DECODERS];

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         decoder_gps_l1ca_init,
  .disable =      decoder_gps_l1ca_disable,
  .process =      decoder_gps_l1ca_process,
  .decoders =     gps_l1ca_decoders,
  .num_decoders = NUM_GPS_L1CA_DECODERS
};

static decoder_interface_list_element_t list_element_gps_l1ca = {
  .interface = &decoder_interface_gps_l1ca,
  .next = 0
};

void decode_gps_l1ca_register(void)
{
  for (u32 i=0; i<NUM_GPS_L1CA_DECODERS; i++) {
    gps_l1ca_decoders[i].active = false;
    gps_l1ca_decoders[i].data = &gps_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l1ca);
}

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data)
{
  (void)channel_info;
  gps_l1ca_decoder_data_t *data = decoder_data;
  nav_msg_init(&data->nav_msg);
}

static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  gps_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit)) {
    /* Update TOW */
    bool bit_val = soft_bit >= 0;
    s32 TOW_ms = nav_msg_update(&data->nav_msg, bit_val);
    s8 bit_polarity = data->nav_msg.bit_polarity;
    if ((TOW_ms >= 0) && (bit_polarity != BIT_POLARITY_UNKNOWN)) {
      if (!tracking_channel_time_sync(channel_info->tracking_channel, TOW_ms,
                                      bit_polarity)) {
        log_warn_sid(channel_info->sid, "TOW set failed");
      }
    }
  }

  /* Check if there is a new nav msg subframe to process. */
  if (!subframe_ready(&data->nav_msg))
    return;

  /* Decode nav data to temporary structure */
  gps_l1ca_decoded_data_t dd;
  s8 ret = process_subframe(&data->nav_msg, channel_info->sid, &dd);
  if (ret <= 0)
    return;


  if (dd.gps_l2c_sv_capability_upd_flag) {
    /* Store new L2C value  */
    log_debug("L2C capabilities received: 0x%x", dd.gps_l2c_sv_capability);
    gps_l2cm_l2c_cap_store(dd.gps_l2c_sv_capability);
  }

  if (dd.iono_corr_upd_flag) {
    /* store new iono parameters */
    log_debug("Iono parameters received");
    gps_iono_params_store(&dd.iono);
  }

  if(dd.ephemeris_upd_flag) {
    /* Decoded a new ephemeris. */
    ephemeris_new(&dd.ephemeris);
    ephemeris_t *eph = ephemeris_get(channel_info->sid);
    if (!eph->valid) {
      log_info_sid(channel_info->sid, "ephemeris is invalid");
    }
  }
}
