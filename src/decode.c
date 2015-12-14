/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h>
#include <ch.h>
#include <assert.h>
#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"

#define NUM_DECODER_CHANNELS  12
#define NUM_GPS_L1_DECODERS   12

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED
} state_t;

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE
} event_t;

typedef struct {
  nav_msg_t nav_msg;
} gps_l1_decoder_data_t;

typedef struct {
  bool active;
  void *data;
} decoder_t;

typedef struct {
  gnss_signal_t sid;
  state_t state;
  u8 tracking_channel;
  decoder_t *decoder;
} decoder_channel_t;

typedef struct {
  void (*init)(decoder_channel_t *d);
  void (*disable)(decoder_channel_t *d);
  void (*process)(decoder_channel_t *d);
  decoder_t *decoders;
  u8 num_decoders;
} decoder_interface_t;

static decoder_channel_t decoder_channels[NUM_DECODER_CHANNELS];

static decoder_t gps_l1_decoders[NUM_GPS_L1_DECODERS];
static gps_l1_decoder_data_t gps_l1_decoder_data[NUM_GPS_L1_DECODERS];

static WORKING_AREA_CCM(wa_decode_thread, 3000);

static msg_t decode_thread(void *arg);
static decoder_channel_t * decoder_channel_get(u8 tracking_channel);
static bool available_decoder_get(const decoder_interface_t *interface,
                                  decoder_t **decoder);
static state_t decoder_channel_state_get(const decoder_channel_t *d);
static bool decoder_active(const decoder_t *decoder);
static void event(decoder_channel_t *d, event_t event);
static const decoder_interface_t * decoder_interface_get(gnss_signal_t sid);

static void decoder_gps_l1_init(decoder_channel_t *d);
static void decoder_gps_l1_disable(decoder_channel_t *d);
static void decoder_gps_l1_process(decoder_channel_t *d);

static const decoder_interface_t decoder_interface_default = {
  .init =         0,
  .disable =      0,
  .process =      0,
  .decoders =     0,
  .num_decoders = 0
};

static const decoder_interface_t decoder_interface_gps_l1 = {
  .init =         decoder_gps_l1_init,
  .disable =      decoder_gps_l1_disable,
  .process =      decoder_gps_l1_process,
  .decoders =     gps_l1_decoders,
  .num_decoders = NUM_GPS_L1_DECODERS
};

void decode_setup(void)
{
  memset(decoder_channels, 0, sizeof(decoder_channels));
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channels[i].state = STATE_DISABLED;
    decoder_channels[i].decoder = 0;
  }

  for (u32 i=0; i<NUM_GPS_L1_DECODERS; i++) {
    gps_l1_decoders[i].active = false;
    gps_l1_decoders[i].data = &gps_l1_decoder_data[i];
  }

  chThdCreateStatic(wa_decode_thread, sizeof(wa_decode_thread),
                    NORMALPRIO-1, decode_thread, NULL);
}

/* Determine if a decoder channel is available for the specified sid */
bool decoder_channel_available(u8 tracking_channel, gnss_signal_t sid)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != STATE_DISABLED)
    return false;

  const decoder_interface_t *interface = decoder_interface_get(sid);
  decoder_t *decoder;
  if (!available_decoder_get(interface, &decoder))
    return false;

  return true;
}

/* Initialize a decoder channel to process telemetry for sid from the specified
   tracking channel. */
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != STATE_DISABLED)
    return false;

  const decoder_interface_t *interface = decoder_interface_get(sid);
  decoder_t *decoder;
  if (!available_decoder_get(interface, &decoder))
    return false;

  /* Set up channel */
  d->tracking_channel = tracking_channel;
  d->sid = sid;
  d->decoder = decoder;

  /* Empty the nav bit FIFO */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    ;
  }

  interface->init(d);
  event(d, EVENT_ENABLE);
  return true;
}

/* Disable the decoder channel associated with the specified tracking channel. */
bool decoder_channel_disable(u8 tracking_channel)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != STATE_ENABLED)
    return false;

  /* Request disable */
  event(d, EVENT_DISABLE_REQUEST);
  return true;
}

static msg_t decode_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decode");

  while (TRUE) {

    for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
      decoder_channel_t *d = &decoder_channels[i];
      state_t state = decoder_channel_state_get(d);
      switch (state) {
      case STATE_ENABLED: {
        const decoder_interface_t *interface = decoder_interface_get(d->sid);
        interface->process(d);
      }
      break;

      case STATE_DISABLE_REQUESTED: {
        const decoder_interface_t *interface = decoder_interface_get(d->sid);
        interface->disable(d);
        event(d, EVENT_DISABLE);
      }
      break;

      default:
        break;
      }
    }

    chThdSleep(MS2ST(1));
  }

  return 0;
}

static decoder_channel_t * decoder_channel_get(u8 tracking_channel)
{
  /* TODO: Decouple tracking / decoder channels somewhat.
   * Just need to make sure that only a single decoder channel can be allocated
   * to a given tracking channel.
   */
  assert(tracking_channel < NUM_DECODER_CHANNELS);
  return &decoder_channels[tracking_channel];
}

static bool available_decoder_get(const decoder_interface_t *interface,
                                  decoder_t **decoder)
{
  /* Search for a free decoder */
  for (u32 j=0; j<interface->num_decoders; j++) {
    if (!decoder_active(&interface->decoders[j])) {
      *decoder = &interface->decoders[j];
      return true;
    }
  }

  return false;
}

static state_t decoder_channel_state_get(const decoder_channel_t *d)
{
  state_t state = d->state;
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  return state;
}

static bool decoder_active(const decoder_t *decoder)
{
  bool active = decoder->active;
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  return active;
}

static void event(decoder_channel_t *d, event_t event)
{
  switch (event) {
  case EVENT_ENABLE: {
    assert(d->state == STATE_DISABLED);
    assert(d->decoder->active == false);
    d->decoder->active = true;
    /* Sequence point for enable is setting channel state = STATE_ENABLED */
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    d->state = STATE_ENABLED;
  }
  break;

  case EVENT_DISABLE_REQUEST: {
    assert(d->state == STATE_ENABLED);
    d->state = STATE_DISABLE_REQUESTED;
  }
  break;

  case EVENT_DISABLE: {
    assert(d->state == STATE_DISABLE_REQUESTED);
    assert(d->decoder->active == true);
    /* Sequence point for disable is setting channel state = STATE_DISABLED
     * and/or decoder active = false (order of these two is irrelevant here) */
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    d->decoder->active = false;
    d->state = STATE_DISABLED;
  }
  break;
  }
}

static const decoder_interface_t * decoder_interface_get(gnss_signal_t sid)
{
  switch (sid.constellation) {
  case CONSTELLATION_GPS: {
    switch (sid.band) {
    case BAND_L1:
      return &decoder_interface_gps_l1;
    }
  }
  break;
  }

  return &decoder_interface_default;
}

/* GPS L1 decoder */
static void decoder_gps_l1_init(decoder_channel_t *d)
{
  gps_l1_decoder_data_t *data = d->decoder->data;
  nav_msg_init(&data->nav_msg);
}

static void decoder_gps_l1_disable(decoder_channel_t *d)
{
  (void)d;
}

static void decoder_gps_l1_process(decoder_channel_t *d)
{
  gps_l1_decoder_data_t *data = d->decoder->data;

  /* Process incoming nav bits */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    /* Update TOW */
    bool bit_val = soft_bit >= 0;
    s32 TOW_ms = nav_msg_update(&data->nav_msg, bit_val);
    s8 bit_polarity = data->nav_msg.bit_polarity;
    if ((TOW_ms >= 0) && (bit_polarity != BIT_POLARITY_UNKNOWN)) {
      if (!tracking_channel_time_sync(d->tracking_channel, TOW_ms,
                                      bit_polarity)) {
        char buf[SID_STR_LEN_MAX];
        sid_to_string(buf, sizeof(buf), d->sid);
        log_warn("%s TOW set failed", buf);
      }
    }
  }

  /* Check if there is a new nav msg subframe to process.
   * TODO: move this into a function */
  tracking_channel_t *ch = &tracking_channel[d->tracking_channel];
  if ((ch->state != TRACKING_RUNNING) ||
      !subframe_ready(&data->nav_msg))
    return;

  /* Decode ephemeris to temporary struct */
  ephemeris_t e = {.sid = d->sid};
  s8 ret = process_subframe(&data->nav_msg, &e);;

  if (ret <= 0)
    return;

  /* Decoded a new ephemeris. */
  ephemeris_new(&e);

  ephemeris_t *eph = ephemeris_get(d->sid);
  if (!eph->healthy) {
    char buf[SID_STR_LEN_MAX];
    sid_to_string(buf, sizeof(buf), d->sid);
    log_info("%s unhealthy", buf);
  } else {
    msg_ephemeris_t msg;
    pack_ephemeris(eph, &msg);
    sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *)&msg);
  }
}
