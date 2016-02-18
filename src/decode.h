/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_H
#define SWIFTNAV_DECODE_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

/** \addtogroup decoding
 * \{ */

typedef void decoder_data_t;

/** Instance of a decoder implementation. */
typedef struct {
  bool active;          /**< true if decoder is in use. */
  decoder_data_t *data; /**< Pointer to data used by decoder instance. */
} decoder_t;

/** Info associated with a decoder channel. */
typedef struct {
  gnss_signal_t sid;    /**< Current signal being decoded. */
  u8 tracking_channel;  /**< Associated tracking channel. */
} decoder_channel_info_t;

/** Decoder interface function template. */
typedef void (*decoder_interface_function_t)(
                 const decoder_channel_info_t *channel_info,
                 decoder_data_t *decoder_data);

/** Interface to a decoder implementation. */
typedef struct {
  enum code code;   /**< Code type for which the implementation may be used. */
  /** Init function. Called to set up decoder instance when decoding begins. */
  decoder_interface_function_t init;
  /** Disable function. Called when decoding stops. */
  decoder_interface_function_t disable;
  /** Process function. Called periodically. Should be used to receive and
   * decode navigation message bits. */
  decoder_interface_function_t process;
  decoder_t *decoders;  /**< Array of decoder instances used by this interface. */
  u8 num_decoders;      /**< Number of decoder instances in decoders array. */
} decoder_interface_t;

/** List element passed to decoder_interface_register(). */
typedef struct decoder_interface_list_element_t {
  const decoder_interface_t *interface;
  struct decoder_interface_list_element_t *next;
} decoder_interface_list_element_t;

/** \} */

void decode_setup(void);
void decoder_interface_register(decoder_interface_list_element_t *element);

bool decoder_channel_available(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_disable(u8 tracking_channel);

#endif
