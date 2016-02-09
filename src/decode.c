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

#include <libswiftnav/logging.h>
#include <ch.h>
#include <assert.h>
#include "track.h"
#include "decode.h"

/** \defgroup decoding Decoding
 * Receive data bits from tracking channels and decode navigation messages.
 * \{ */

/*
 * Channel States and Transitions:
 *
 * DISABLED
 *  - Channel will not be accessed by the decoder thread. Set configuration,
 *    associate an inactive decoder instance, and then issue *EVENT_ENABLE* to
 *    set the decoder active and transition state to ENABLED.
 *
 * ENABLED
 *  - Channel is running and may be accessed by the decoder thread. Only
 *    allowed external action is an atomic state transition from ENABLED to
 *    DISABLE_REQUEST by issuing *EVENT_DISABLE_REQUEST*.
 *
 * DISABLE_REQUEST
 *  - Channel is running but will be disabled by the decoder thread imminently.
 *    No external action is allowed. Decoder thread should issue *EVENT_DISABLE*
 *    to set the decoder inactive and transition state to DISABLED.
 *
 *
 *
 * Notes on atomicity and thread safety:
 * - When enabling a decoder channel, the only sequence point is setting channel
 *   state = STATE_ENABLED. Parameters may be set in any order, and it does not
 *   matter when the decoder instance is set to active, as long as all of this
 *   is completed before channel state = STATE_ENABLED. Channel state gates all
 *   accesses to a decoder channel and associated decoder instance from the
 *   decoder thread.
 *
 * - When disabling a decoder channel (from the decoder thread), the only
 *   sequence points are releasing the decoder channel (channel state =
 *   STATE_DISABLED) and the decoder instance (active = false). All accesses
 *   to the these structures must complete before the corresponding release.
 *   It does not matter in which order the two structures are released as they
 *   are allocated independently when initializing decoding.
 */

#define NUM_DECODER_CHANNELS  12

typedef enum {
  DECODER_CHANNEL_STATE_DISABLED,
  DECODER_CHANNEL_STATE_ENABLED,
  DECODER_CHANNEL_STATE_DISABLE_REQUESTED
} decoder_channel_state_t;

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE
} event_t;

/** Top-level generic decoder channel. */
typedef struct {
  decoder_channel_state_t state;  /**< State of this channel. */
  decoder_channel_info_t info;    /**< Info associated with this channel. */
  decoder_t *decoder;             /**< Associated decoder instance. */
} decoder_channel_t;

static decoder_interface_list_element_t *decoder_interface_list = 0;
static decoder_channel_t decoder_channels[NUM_DECODER_CHANNELS];

static WORKING_AREA_CCM(wa_decode_thread, 3000);

static void decode_thread(void *arg);
static const decoder_interface_t * decoder_interface_get(gnss_signal_t sid);
static decoder_channel_t * decoder_channel_get(u8 tracking_channel);
static bool available_decoder_get(const decoder_interface_t *interface,
                                  decoder_t **decoder);
static decoder_channel_state_t decoder_channel_state_get(const decoder_channel_t *d);
static bool decoder_active(const decoder_t *decoder);
static void interface_function(decoder_channel_t *d,
                               decoder_interface_function_t func);
static void event(decoder_channel_t *d, event_t event);

static const decoder_interface_t decoder_interface_default = {
  .code =         CODE_INVALID,
  .init =         0,
  .disable =      0,
  .process =      0,
  .decoders =     0,
  .num_decoders = 0
};

/** Set up the decoding module. */
void decode_setup(void)
{
  for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
    decoder_channels[i].state = DECODER_CHANNEL_STATE_DISABLED;
    decoder_channels[i].decoder = 0;
  }

  chThdCreateStatic(wa_decode_thread, sizeof(wa_decode_thread),
                    NORMALPRIO-1, decode_thread, NULL);
}

/** Register a decoder interface to enable decoding for a constellation / band.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void decoder_interface_register(decoder_interface_list_element_t *element)
{
  /* p_next = address of next pointer which must be updated */
  decoder_interface_list_element_t **p_next = &decoder_interface_list;

  while (*p_next != 0)
    p_next = &(*p_next)->next;

  element->next = 0;
  *p_next = element;
}

/** Determine if a decoder channel is available for the specified tracking
 * channel and sid.
 *
 * \param tracking_channel  Tracking channel to use.
 * \param sid               Signal to be decoded.
 *
 * \return true if a decoder channel is available, false otherwise.
 */
bool decoder_channel_available(u8 tracking_channel, gnss_signal_t sid)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != DECODER_CHANNEL_STATE_DISABLED)
    return false;

  const decoder_interface_t *interface = decoder_interface_get(sid);
  decoder_t *decoder;
  if (!available_decoder_get(interface, &decoder))
    return false;

  return true;
}

/** Initialize a decoder channel to process telemetry for sid from the
 * specified tracking channel.
 *
 * \param tracking_channel  Tracking channel to use.
 * \param sid               Signal to be decoded.
 *
 * \return true if a decoder channel was initialized, false otherwise.
 */
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != DECODER_CHANNEL_STATE_DISABLED)
    return false;

  const decoder_interface_t *interface = decoder_interface_get(sid);
  decoder_t *decoder;
  if (!available_decoder_get(interface, &decoder))
    return false;

  /* Set up channel */
  d->info.tracking_channel = tracking_channel;
  d->info.sid = sid;
  d->decoder = decoder;

  /* Empty the nav bit FIFO */
  s8 soft_bit;
  while (tracking_channel_nav_bit_get(d->info.tracking_channel, &soft_bit)) {
    ;
  }

  interface_function(d, interface->init);
  event(d, EVENT_ENABLE);
  return true;
}

/** Disable the decoder channel associated with the specified
 * tracking channel.
 *
 * \param tracking_channel  Tracking channel to use.
 *
 * \return true if a decoder channel was disabled, false otherwise.
 */
bool decoder_channel_disable(u8 tracking_channel)
{
  decoder_channel_t *d = decoder_channel_get(tracking_channel);
  if (decoder_channel_state_get(d) != DECODER_CHANNEL_STATE_ENABLED)
    return false;

  /* Request disable */
  event(d, EVENT_DISABLE_REQUEST);
  return true;
}

static void decode_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("decode");

  while (TRUE) {

    for (u32 i=0; i<NUM_DECODER_CHANNELS; i++) {
      decoder_channel_t *d = &decoder_channels[i];
      switch (decoder_channel_state_get(d)) {
      case DECODER_CHANNEL_STATE_ENABLED: {
        const decoder_interface_t *interface = decoder_interface_get(d->info.sid);
        interface_function(d, interface->process);
      }
      break;

      case DECODER_CHANNEL_STATE_DISABLE_REQUESTED: {
        const decoder_interface_t *interface = decoder_interface_get(d->info.sid);
        interface_function(d, interface->disable);
        event(d, EVENT_DISABLE);
      }
      break;

      default:
        break;
      }
    }

    chThdSleep(MS2ST(1));
  }
}

/** Retrieve the decoder interface for the specified sid.
 *
 * \param sid               Signal to be decoded.
 *
 * \return Associated decoder interface. May be the default interface.
 */
static const decoder_interface_t * decoder_interface_get(gnss_signal_t sid)
{
  const decoder_interface_list_element_t *e = decoder_interface_list;
  while (e != 0) {
    const decoder_interface_t *interface = e->interface;
    if (interface->code == sid.code) {
      return interface;
    }
    e = e->next;
  }

  return &decoder_interface_default;
}

/** Retrieve the decoder channel associated with the specified tracking channel.
 *
 * \param tracking_channel  Tracking channel to use.
 *
 * \return Associated decoder channel.
 */
static decoder_channel_t * decoder_channel_get(u8 tracking_channel)
{
  /* TODO: Decouple tracking / decoder channels somewhat.
   * Just need to make sure that only a single decoder channel can be allocated
   * to a given tracking channel.
   */
  assert(tracking_channel < NUM_DECODER_CHANNELS);
  return &decoder_channels[tracking_channel];
}

/** Find an inactive decoder instance for the specified decoder interface.
 *
 * \param interface       Decoder interface to use.
 * \param decoder         Output inactive decoder instance.
 *
 * \return true if *decoder points to an inactive decoder instance,
 * false otherwise.
 */
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

/** Return the state of a decoder channel.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param d         Decoder channel to use.
 *
 * \return state of the decoder channel.
 */
static decoder_channel_state_t decoder_channel_state_get(const decoder_channel_t *d)
{
  decoder_channel_state_t state = d->state;
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  return state;
}

/** Return the state of a decoder instance.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param decoder   Decoder to use.
 *
 * \return true if the decoder is active, false if inactive.
 */
static bool decoder_active(const decoder_t *decoder)
{
  bool active = decoder->active;
  asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
  return active;
}

/** Execute an interface function on a decoder channel.
 *
 * \param d         Decoder channel to use.
 * \param func      Interface function to execute.
 */
static void interface_function(decoder_channel_t *d,
                               decoder_interface_function_t func)
{
  return func(&d->info, d->decoder->data);
}

/** Update the state of a decoder channel and its associated decoder instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param d       Decoder channel to use.
 * \param event   Event to process.
 */
static void event(decoder_channel_t *d, event_t event)
{
  switch (event) {
  case EVENT_ENABLE: {
    assert(d->state == DECODER_CHANNEL_STATE_DISABLED);
    assert(d->decoder->active == false);
    d->decoder->active = true;
    /* Sequence point for enable is setting channel state = STATE_ENABLED */
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    d->state = DECODER_CHANNEL_STATE_ENABLED;
  }
  break;

  case EVENT_DISABLE_REQUEST: {
    assert(d->state == DECODER_CHANNEL_STATE_ENABLED);
    d->state = DECODER_CHANNEL_STATE_DISABLE_REQUESTED;
  }
  break;

  case EVENT_DISABLE: {
    assert(d->state == DECODER_CHANNEL_STATE_DISABLE_REQUESTED);
    assert(d->decoder->active == true);
    /* Sequence point for disable is setting channel state = STATE_DISABLED
     * and/or decoder active = false (order of these two is irrelevant here) */
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    d->decoder->active = false;
    d->state = DECODER_CHANNEL_STATE_DISABLED;
  }
  break;
  }
}

/** \} */
