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

#include <string.h>

#include <libsbp/settings.h>
#include <libswiftnav/logging.h>

#include "peripherals/usart.h"
#include "minIni/minIni.h"
#include "sbp.h"
#include "settings.h"

#define SETTINGS_FILE "config"


static struct setting *settings_head;

static const char const * bool_enum[] = {"False", "True", NULL};
static struct setting_type bool_settings_type;
/* Bool type identifier can't be a constant because its allocated on setup. */
int TYPE_BOOL = 0;

static int float_to_string(const void *priv, char *str, int slen, const void *blob, int blen)
{
  (void)priv;

  switch (blen) {
  case 4:
    return snprintf(str, slen, "%.12g", (double)*(float*)blob);
  case 8:
    return snprintf(str, slen, "%.12g", *(double*)blob);
  }
  return -1;
}

static bool float_from_string(const void *priv, void *blob, int blen, const char *str)
{
  (void)priv;

  switch (blen) {
  case 4:
    return sscanf(str, "%g", (float*)blob) == 1;
  case 8:
    return sscanf(str, "%lg", (double*)blob) == 1;
  }
  return false;
}

static int int_to_string(const void *priv, char *str, int slen, const void *blob, int blen)
{
  (void)priv;

  switch (blen) {
  case 1:
    return snprintf(str, slen, "%hhd", *(s8*)blob);
  case 2:
    return snprintf(str, slen, "%hd", *(s16*)blob);
  case 4:
    return snprintf(str, slen, "%ld", *(s32*)blob);
  }
  return -1;
}

static bool int_from_string(const void *priv, void *blob, int blen, const char *str)
{
  (void)priv;

  switch (blen) {
  case 1: {
    s16 tmp;
    /* Newlib's crappy sscanf doesn't understand %hhd */
    if (sscanf(str, "%hd", &tmp) == 1) {
      *(s8*)blob = tmp;
      return true;
    }
    return false;
  }
  case 2:
    return sscanf(str, "%hd", (s16*)blob) == 1;
  case 4:
    return sscanf(str, "%ld", (s32*)blob) == 1;
  }
  return false;
}

static int str_to_string(const void *priv, char *str, int slen, const void *blob, int blen)
{
  (void)priv;
  if (blen < slen)
    slen = blen;
  strncpy(str, blob, slen);
  return strnlen(str, slen);
}

static bool str_from_string(const void *priv, void *blob, int blen, const char *str)
{
  (void)priv;
  strncpy(blob, str, blen);
  return true;
}

static int enum_to_string(const void *priv, char *str, int slen, const void *blob, int blen)
{
  const char * const *enumnames = priv;
  if (blen != sizeof(u8))
    asm("bkpt");
  int index = *(u8*)blob;
  strncpy(str, enumnames[index], slen);
  return strlen(str);
}

static bool enum_from_string(const void *priv, void *blob, int blen, const char *str)
{
  const char * const *enumnames = priv;
  int i;

  if (blen != sizeof(u8))
    asm("bkpt");

  for (i = 0; enumnames[i] && (strcmp(str, enumnames[i]) != 0); i++)
    ;

  if (!enumnames[i])
    return false;

  *(u8*)blob = i;

  return true;
}

int enum_format_type(const void *priv, char *str, int len)
{
  int i = 5;
  strncpy(str, "enum:", len);
  for (const char * const *enumnames = priv; *enumnames; enumnames++)
    i = snprintf(str, len-i, "%s%s,", str, *enumnames);
  str[i-1] = '\0';
  return i;
}

static struct setting_type type_string = {
  str_to_string, str_from_string, NULL, NULL, NULL,
};

static struct setting_type type_float = {
  float_to_string, float_from_string, NULL, NULL, &type_string,
};

static const struct setting_type type_int = {
  int_to_string, int_from_string, NULL, NULL, &type_float,
};

static void settings_save_callback(u16 sender_id, u8 len, u8 msg[], void* context);
static void settings_write_callback(u16 sender_id, u8 len, u8 msg[], void* context);
static void settings_read_callback(u16 sender_id, u8 len, u8 msg[], void* context);
static void settings_read_by_index_callback(u16 sender_id, u8 len, u8 msg[], void* context);

int settings_type_register_enum(const char * const enumnames[], struct setting_type *type)
{
  int i;
  struct setting_type *t;
  type->to_string = enum_to_string;
  type->from_string = enum_from_string;
  type->format_type = enum_format_type;
  type->priv = enumnames;
  for (i = 0, t = (struct setting_type*)&type_int; t->next; t = t->next, i++)
    ;
  i++;
  t->next = type;
  return i;
}

void settings_setup(void)
{
  TYPE_BOOL = settings_type_register_enum(bool_enum, &bool_settings_type);

  static sbp_msg_callbacks_node_t settings_save_node;
  sbp_register_cbk(
    SBP_MSG_SETTINGS_SAVE,
    &settings_save_callback,
    &settings_save_node
  );
  static sbp_msg_callbacks_node_t settings_write_node;
  sbp_register_cbk(
    SBP_MSG_SETTINGS_WRITE,
    &settings_write_callback,
    &settings_write_node
  );
  static sbp_msg_callbacks_node_t settings_read_node;
  sbp_register_cbk(
    SBP_MSG_SETTINGS_READ_REQ,
    &settings_read_callback,
    &settings_read_node
  );
  static sbp_msg_callbacks_node_t settings_read_by_index_node;
  sbp_register_cbk(
    SBP_MSG_SETTINGS_READ_BY_INDEX_REQ,
    &settings_read_by_index_callback,
    &settings_read_by_index_node
  );
}

void settings_register(struct setting *setting, enum setting_types type)
{
  struct setting *s;
  const struct setting_type *t = &type_int;

  for (int i = 0; t && (i < type); i++, t = t->next)
    ;
  /* FIXME Abort if type is NULL */
  setting->type = t;

  if (!settings_head) {
    settings_head = setting;
  } else {
    for (s = settings_head; s->next; s = s->next) {
      if ((strcmp(s->section, setting->section) == 0) &&
          (strcmp(s->next->section, setting->section) != 0))
        break;
    }
    setting->next = s->next;
    s->next = setting;
  }
  char buf[128];
  ini_gets(setting->section, setting->name, "", buf, sizeof(buf), SETTINGS_FILE);
  if (buf[0] == 0) {
    setting->type->to_string(setting->type->priv, buf, sizeof(buf),
                             setting->addr, setting->len);
    setting->notify(setting, buf);
  } else {
    *strchr(buf, '\n') = '\0';
    setting->dirty = setting->notify(setting, buf);
  }
}

static struct setting *settings_lookup(const char *section, const char *setting)
{
  for (struct setting *s = settings_head; s; s = s->next)
    if ((strcmp(s->section, section)  == 0) &&
        (strcmp(s->name, setting) == 0))
      return s;
  return NULL;
}

bool settings_default_notify(struct setting *s, const char *val)
{
  return s->type->from_string(s->type->priv, s->addr, s->len, val);
}

bool settings_read_only_notify(struct setting *s, const char *val)
{
  (void)s; (void)val;
  return true;
}

static int settings_format_setting(struct setting *s, char *buf, int len)
{
  int buflen;

  /* build and send reply */
  strncpy(buf, s->section, len);
  buflen = strlen(s->section) + 1;
  strncpy(buf + buflen, s->name, len - buflen);
  buflen += strlen(s->name) + 1;
  buflen += s->type->to_string(s->type->priv,
                               buf + buflen, len - buflen,
                               s->addr, s->len);
  buf[buflen++] = '\0';
  if (s->type->format_type != NULL)
    buflen += s->type->format_type(s->type->priv, buf + buflen, len - buflen);

  return buflen;
}

static void settings_write_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender");
    return;
  }

  static struct setting *s = NULL;
  const char *section = NULL, *setting = NULL, *value = NULL;

  if (len == 0) {
    log_error("Error in settings write message");
    return;
  }

  if (msg[len-1] != '\0') {
    log_error("Error in settings write message");
    return;
  }

  /* Extract parameters from message:
   * 3 null terminated strings: section, setting and value
   */
  section = (const char *)msg;
  for (int i = 0, tok = 0; i < len; i++) {
    if (msg[i] == '\0') {
      tok++;
      switch (tok) {
      case 1:
        setting = (const char *)&msg[i+1];
        break;
      case 2:
        if (i + 1 < len)
          value = (const char *)&msg[i+1];
        break;
      case 3:
        if (i == len-1)
          break;
      default:
        log_error("Error in settings write message");
        return;
      }
    }
  }

  if (value == NULL) {
    log_error("Error in settings write message");
    return;
  }

  s = settings_lookup(section, setting);
  if (s == NULL) {
    log_error("Error in settings write message");
    return;
  }

  /* This is an assignment, call notify function */
  if (!s->notify(s, value)) {
    log_error("Error in settings write message");
    return;
  }
  s->dirty = true;
  return;
}

static void settings_read_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender");
    return;
  }

  static struct setting *s = NULL;
  const char *section = NULL, *setting = NULL;
  char buf[256];
  u8 buflen;

  if (len == 0) {
    log_error("Error in settings read message");
    return;
  }

  if (msg[len-1] != '\0') {
    log_error("Error in settings read message");
    return;
  }

  /* Extract parameters from message:
   * 2 null terminated strings: section, and setting
   */
  section = (const char *)msg;
  for (int i = 0, tok = 0; i < len; i++) {
    if (msg[i] == '\0') {
      tok++;
      switch (tok) {
      case 1:
        setting = (const char *)&msg[i+1];
        break;
      case 2:
        if (i == len-1)
          break;
      default:
        log_error("Error in settings read message");
        return;
      }
    }
  }

  s = settings_lookup(section, setting);
  if (s == NULL) {
    log_error("Error in settings read message");
    return;
  }

  buflen = settings_format_setting(s, buf, sizeof(buf));
  sbp_send_msg(SBP_MSG_SETTINGS_READ_RESP, buflen, (void*)buf);
  return;
}

static void settings_read_by_index_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender");
    return;
  }

  struct setting *s = settings_head;
  char buf[256];
  u8 buflen = 0;

  if (len != 2) {
    log_error("Invalid length for settings read by index!");
    return;
  }
  u16 index = (msg[1] << 8) | msg[0];

  for (int i = 0; (i < index) && s; i++, s = s->next)
    ;

  if (s == NULL) {
    sbp_send_msg(SBP_MSG_SETTINGS_READ_BY_INDEX_DONE, 0, NULL);
    return;
  }

  /* build and send reply */
  buf[buflen++] = msg[0];
  buf[buflen++] = msg[1];
  buflen += settings_format_setting(s, buf + buflen, sizeof(buf) - buflen);
  sbp_send_msg(SBP_MSG_SETTINGS_READ_BY_INDEX_RESP, buflen, (void*)buf);
}

static void settings_save_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  int f = cfs_open(SETTINGS_FILE, CFS_WRITE);
  const char *sec = NULL;
  char buf[128];
  int i;

  (void)sender_id; (void) context; (void)len; (void)msg;

  if (f == -1) {
    log_error("Error opening config file!");
    return;
  }

  for (struct setting *s = settings_head; s; s = s->next) {
    /* Skip unchanged parameters */
    if (!s->dirty)
      continue;

    if ((sec == NULL) || (strcmp(s->section, sec) != 0)) {
      /* New section, write section header */
      sec = s->section;
      i = snprintf(buf, sizeof(buf), "[%s]\n", sec);
      if (cfs_write(f, buf, i) != i) {
        log_error("Error writing to config file!");
      }
    }

    /* Write setting */
    i = snprintf(buf, sizeof(buf), "%s=", s->name);
    i += s->type->to_string(s->type->priv, &buf[i], sizeof(buf) - i - 1, s->addr, s->len);
    buf[i++] = '\n';
    if (cfs_write(f, buf, i) != i) {
      log_error("Error writing to config file!");
    }
  }

  cfs_close(f);
  log_info("Wrote settings to config file.");
}

