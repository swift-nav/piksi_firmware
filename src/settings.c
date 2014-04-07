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

#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"

#include <string.h>
#include <stdio.h>

/** \addtogroup io
 * \{ */

settings_t settings /*__attribute__ ((section(".settings_area"))) */=
/* Default settings: */
{
  .settings_valid = VALID,

  .ftdi_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_FTDI,
    .message_mask = 0xFFFF,
  },
  .uarta_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_TTL,
    .message_mask = 0x40,
  },
  .uartb_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_TTL,
    .message_mask = 0xFF00
  },
};

/** \} */

static struct setting *settings_head;

static int float_to_string(void *priv, char *str, int slen, const void *blob, int blen)
{
  (void)priv;

  switch (blen) {
  case 4:
    return snprintf(str, slen, "%e", (double)*(float*)blob);
  case 8:
    return snprintf(str, slen, "%e", *(double*)blob);
  }
  return -1;
}

static bool float_from_string(void *priv, void *blob, int blen, const char *str)
{
  (void)priv;

  switch (blen) {
  case 4:
    return sscanf(str, "%f", (float*)blob) == 1;
  case 8:
    return sscanf(str, "%lf", (double*)blob) == 1;
  }
  return false;
}

static int int_to_string(void *priv, char *str, int slen, const void *blob, int blen)
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

static bool int_from_string(void *priv, void *blob, int blen, const char *str)
{
  (void)priv;

  switch (blen) {
  case 1:
    return sscanf(str, "%hhd", (s8*)blob) == 1;
  case 2:
    return sscanf(str, "%hd", (s16*)blob) == 1;
  case 4:
    return sscanf(str, "%ld", (s32*)blob) == 1;
  }
  return false;
}

static struct setting_type type_float = {
  float_to_string, float_from_string, NULL, NULL,
};

static const struct setting_type type_int = {
  int_to_string, int_from_string, NULL, &type_float,
};

static void settings_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context);

void settings_setup(void)
{
  static sbp_msg_callbacks_node_t settings_msg_node;
  sbp_register_cbk(
    MSG_SETTINGS,
    &settings_msg_callback,
    &settings_msg_node
  );
  SETTING("ftdi_uart", "baudrate", settings.ftdi_usart.baud_rate, TYPE_INT);
  SETTING("uarta_uart", "baudrate", settings.uarta_usart.baud_rate, TYPE_INT);
  SETTING("uartb_uart", "baudrate", settings.uartb_usart.baud_rate, TYPE_INT);
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
    return;
  }
  for (s = settings_head; s->next; s = s->next)
    ;
  s->next = setting;
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

static void settings_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void) context;

  static struct setting *s = NULL;
  const char *section = NULL, *setting = NULL, *value = NULL;
  char buf[256];
  u8 buflen;

  if (len == 0) {
    /* Empty message is for parameter enumeration */
    if (s && s->next)
      s = s->next;
    else
      s = settings_head;
  } else {
    /* Extract parameters from message:
     * 2 or 3 null terminated strings: section, setting and (optional) value
     * If value is present the message is an assignment.
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
          value = (const char *)&msg[i+1];
          break;
        case 3:
          if (i == len-1)
            break;
        default:
          goto error;
        }
      }
    }
    s = settings_lookup(section, setting);
  }
  if (s == NULL)
    goto error;

  if (value != NULL) {
    /* This is an assignment, call notify function */
    if (!s->notify(s, value))
      goto error;
  }

  /* build and send reply */
  strncpy(buf, s->section, sizeof(buf));
  buflen = strlen(s->section) + 1;
  strncpy(buf + buflen, s->name, sizeof(buf) - buflen);
  buflen += strlen(s->name) + 1;
  buflen += s->type->to_string(s->type->priv,
                               buf + buflen, sizeof(buf) - buflen,
                               s->addr, s->len);
  sbp_send_msg(MSG_SETTINGS, buflen, (void*)buf);
  return;

error:
  printf("Error in settings read message\n");
}

