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


enum setting_types {
  TYPE_INT,
  TYPE_FLOAT,
  TYPE_STRING,
};
extern int TYPE_BOOL;

struct setting_type {
  int (*to_string)(const void *priv, char *str, int slen, const void *blob, int blen);
  bool (*from_string)(const void *priv, void *blob, int len, const char *str);
  int (*format_type)(const void *priv, char *str, int len);
  const void *priv;
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

#define READ_ONLY_PARAMETER(section, name, var, type) \
  SETTING_NOTIFY(section, name, var, type, settings_read_only_notify)

void settings_setup(void);
int settings_type_register_enum(const char * const enumnames[], struct setting_type *type);
void settings_register(struct setting *s, enum setting_types type);
bool settings_default_notify(struct setting *setting, const char *val);
bool uarta_baudrate_notify(struct setting *setting, const char *val);
bool settings_read_only_notify(struct setting *setting, const char *val);

#endif  /* SWIFTNAV_SETTINGS_H */

