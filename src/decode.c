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

#define NUM_DECODER_CHANNELS 12

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED
} state_t;

typedef struct {

} default_decoder_t;

typedef struct {
  nav_msg_t nav_msg;
} gps_l1_decoder_t;

typedef struct {
  gnss_signal_t sid;
  state_t state;
  u8 tracking_channel;
  union {
    default_decoder_t *default_decoder;
    gps_l1_decoder_t *gps_l1_decoder;
  };
} decoder_channel_t;

typedef struct {
  bool (*available)(gnss_signal_t sid);
  bool (*init)(decoder_channel_t *d);
  bool (*disable)(decoder_channel_t *d);
  bool (*process)(decoder_channel_t *d);
} decoder_interface_t;

static gps_l1_decoder_t gps_l1_decoder_channels[NUM_DECODER_CHANNELS];
static decoder_channel_t decoder_channels[NUM_DECODER_CHANNELS];

static WORKING_AREA_CCM(wa_decode_thread, 3000);

static msg_t decode_thread(void *arg);
static state_t state_get(const decoder_channel_t *d);
static void state_set(decoder_channel_t *d, state_t state);
static const decoder_interface_t * decoder_interface_get(gnss_signal_t sid);

static bool decoder_default_available(gnss_signal_t sid);
static bool decoder_default_init(decoder_channel_t *d);
static bool decoder_default_disable(decoder_channel_t *d);
static bool decoder_default_process(decoder_channel_t *d);

static bool decoder_gps_l1_available(gnss_signal_t sid);
static bool decoder_gps_l1_init(decoder_channel_t *d);
static bool decoder_gps_l1_disable(decoder_channel_t *d);
static bool decoder_gps_l1_process(decoder_channel_t *d);

static const decoder_interface_t decoder_interface_default = {
  .available =  decoder_default_available,
  .init =       decoder_default_init,
  .disable =    decoder_default_disable,
  .process =    decoder_default_process
};

static const decoder_interface_t decoder_interface_gps_l1 = {
  .available =  decoder_gps_l1_available,
  .init =       decoder_gps_l1_init,
  .disable =    decoder_gps_l1_disable,
  .process =    decoder_gps_l1_process
};

void decode_setup(void)
{
  memset(decoder_channels, 0, sizeof(decoder_channels));
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channels[i].state = STATE_DISABLED;
  }

  chThdCreateStatic(wa_decode_thread, sizeof(wa_decode_thread),
                    NORMALPRIO-1, decode_thread, NULL);
}

static msg_t decode_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decode");

  while (TRUE) {

    for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
      decoder_channel_t *d = &decoder_channels[i];
      state_t state = state_get(d);
      switch (state) {
      case STATE_ENABLED: {
        const decoder_interface_t *interface = decoder_interface_get(d->sid);
        interface->process(d);
      }
      break;

      case STATE_DISABLE_REQUESTED: {
        const decoder_interface_t *interface = decoder_interface_get(d->sid);
        interface->disable(d);
        state_set(d, STATE_DISABLED);
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
/* Determine if a decoder channel is available for the specified sid */
bool decoder_channel_available(gnss_signal_t sid)
{
  /* Search for a free channel */
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channel_t *d = &decoder_channels[i];
    state_t state = state_get(d);
    if (state == STATE_DISABLED) {

      const decoder_interface_t *interface = decoder_interface_get(d->sid);
      return interface->available(sid);
    }
  }

  return false;
}

/* Initialize a decoder channel to process telemetry for sid from the specified
   tracking channel. */
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid)
{
  /* Search for a free channel */
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channel_t *d = &decoder_channels[i];
    state_t state = state_get(d);
    if (state == STATE_DISABLED) {
      d->tracking_channel = tracking_channel;
      d->sid = sid;

      const decoder_interface_t *interface = decoder_interface_get(d->sid);
      if (!interface->init(d))
        return false;

      state_set(d, STATE_ENABLED);
      return true;
    }
  }

  return false;
}

/* Disable the decoder channel associated with the specified tracking channel. */
bool decoder_channel_disable(u8 tracking_channel)
{
  /* Search for channel */
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channel_t *d = &decoder_channels[i];
    state_t state = state_get(d);
    if (state == STATE_ENABLED) {
      if (d->tracking_channel == tracking_channel) {
        /* Request disable */
        state_set(d, STATE_DISABLE_REQUESTED);
        return true;
      }
    }
  }

  return false;
}

static state_t state_get(const decoder_channel_t *d)
{
  state_t state = d->state;
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  return state;
}

static void state_set(decoder_channel_t *d, state_t state)
{
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  d->state = state;
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

/* Default decoder */
static bool decoder_default_available(gnss_signal_t sid)
{
  (void)sid;
  return true;
}

static bool decoder_default_init(decoder_channel_t *d)
{
  d->default_decoder = 0;
  /* Empty the nav bit FIFO */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    ;
  }
  return true;
}

static bool decoder_default_disable(decoder_channel_t *d)
{
  (void)d;
  return true;
}

static bool decoder_default_process(decoder_channel_t *d)
{
  /* Process incoming nav bits */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    ;
  }
  return true;
}

/* GPS L1 decoder */
static bool decoder_gps_l1_available(gnss_signal_t sid)
{
  (void)sid;
  return true;
}

static bool decoder_gps_l1_init(decoder_channel_t *d)
{
  d->gps_l1_decoder = &gps_l1_decoder_channels[d->tracking_channel];
  nav_msg_init(&d->gps_l1_decoder->nav_msg, d->sid);
  /* Empty the nav bit FIFO */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    ;
  }
  return true;
}

static bool decoder_gps_l1_disable(decoder_channel_t *d)
{
  (void)d;
  return true;
}

static bool decoder_gps_l1_process(decoder_channel_t *d)
{
  /* Process incoming nav bits */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->tracking_channel, &soft_bit)) {
    /* Update TOW */
    bool bit_val = soft_bit >= 0;
    s32 TOW_ms = nav_msg_update(&d->gps_l1_decoder->nav_msg, bit_val);
    s8 bit_polarity = d->gps_l1_decoder->nav_msg.bit_polarity;
    if ((TOW_ms >= 0) && (bit_polarity != BIT_POLARITY_UNKNOWN)) {
      if (!tracking_channel_time_sync(d->tracking_channel, TOW_ms, bit_polarity))
        log_warn("PRN %d TOW set failed", d->sid.sat+1);
    }
  }

  /* Check if there is a new nav msg subframe to process.
   * TODO: move this into a function */
  tracking_channel_t *ch = &tracking_channel[d->tracking_channel];
  if ((ch->state != TRACKING_RUNNING) ||
      !subframe_ready(&d->gps_l1_decoder->nav_msg))
    return true;

  /* Decode ephemeris to temporary struct */
  ephemeris_t e = {.sid = d->sid};
  s8 ret = process_subframe(&d->gps_l1_decoder->nav_msg, &e);;

  if (ret <= 0)
    return true;

  /* Decoded a new ephemeris. */
  ephemeris_new(&e);

  ephemeris_t *eph = ephemeris_get(d->sid);
  if (!eph->healthy) {
    log_info("PRN %02d unhealthy", d->sid.sat+1);
  } else {
    msg_ephemeris_t msg;
    pack_ephemeris(eph, &msg);
    sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *)&msg);
  }

  return true;
}
