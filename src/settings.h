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

#ifndef SWIFTNAV_SETTINGS_H
#define SWIFTNAV_SETTINGS_H

#include <libswiftnav/common.h>

/** \addtogroup io
 * \{ */

/** Message and baud rate settings for a USART. */
typedef struct {
  enum {
    SBP,
    NMEA,
    RTCM
  } mode; /** Communication mode : Swift Binary Protocol or NMEA */
  u32 baud_rate;
  u16 message_mask;
} usart_settings_t;

/** Message and baud rate settings for all USARTs. */
typedef struct {
  enum {
    VALID = 0,
    /** Settings area is erased, i.e. all 0xFF. */
    INVALID = 0xFF
  } settings_valid;
  usart_settings_t ftdi_usart;
  usart_settings_t uarta_usart;
  usart_settings_t uartb_usart;
} settings_t;

/** \} */

extern settings_t settings;

enum setting_types {
  TYPE_INT,
  TYPE_FLOAT,
  TYPE_STRING,
};

struct setting_type {
  int (*to_string)(void *priv, char *str, int slen, const void *blob, int blen);
  bool (*from_string)(void *priv, void *blob, int len, const char *str);
  void *priv;
  struct setting_type *next;
};

struct setting {
  const char *section;
  const char *name;
  void *addr;
  int len;
  bool (*notify)(struct setting *setting, const char *val);
  struct setting *next;
  const struct setting_type *type;
  bool dirty;
};

#define SETTING_NOTIFY(section, name, var, type, notify) do {         \
  static struct setting setting = \
    {(section), (name), &(var), sizeof(var), (notify), NULL, NULL, false}; \
  settings_register(&(setting), (type)); \
} while(0)

#define SETTING(section, name, var, type) \
  SETTING_NOTIFY(section, name, var, type, settings_default_notify)

void settings_setup(void);
void settings_register(struct setting *s, enum setting_types type);
bool settings_default_notify(struct setting *setting, const char *val);

#endif  /* SWIFTNAV_SETTINGS_H */

