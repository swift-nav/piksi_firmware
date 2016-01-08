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

typedef void decoder_data_t;

typedef struct {
  bool active;
  decoder_data_t *data;
} decoder_t;

typedef struct {
  gnss_signal_t sid;
  u8 tracking_channel;
} decoder_channel_info_t;

typedef void (*decoder_interface_function_t)(
                 const decoder_channel_info_t *channel_info,
                 decoder_data_t *decoder_data);

typedef struct {
  gnss_signal_t sid;
  decoder_interface_function_t init;
  decoder_interface_function_t disable;
  decoder_interface_function_t process;
  decoder_t *decoders;
  u8 num_decoders;
} decoder_interface_t;

typedef struct decoder_interface_list_element_t {
  const decoder_interface_t *interface;
  struct decoder_interface_list_element_t *next;
} decoder_interface_list_element_t;

void decode_setup(void);
void decoder_interface_register(decoder_interface_list_element_t *element);

bool decoder_channel_available(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_init(u8 tracking_channel, gnss_signal_t sid);
bool decoder_channel_disable(u8 tracking_channel);

#endif
