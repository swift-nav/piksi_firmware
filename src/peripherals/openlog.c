/*
 * Copyright (C) 2012-2015 Swift Navigation Inc.
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
#include <stdio.h>

#include <ch.h>

#include <libopencm3/stm32/f4/usart.h>

#include <libswiftnav/logging.h>

#include "openlog.h"
#include "../settings.h"

/** \addtogroup peripherals
 * \{ */

/** \defgroup openlog OpenLog
 * Interface to the OpenLog open source SD data logger
 *
 * \{ */

#define xstr(s) str(s)
#define str(s) #s

/** 3x ctrl-z */
#define ESCAPE_SEQUENCE_STRING                "\x1A\x1A\x1A"
/** string sent to execute a command */
#define RETURN_STRING                         "\r\n"
/** prompt returned on command success */
#define COMMAND_PROMPT_STRING                 "\n>\n"
/** prompt returned on command error */
#define COMMAND_PROMPT_ERROR_STRING           "\n!>\n"
/** prompt returned when entering data mode */
#define DATA_PROMPT_STRING                    "<"
#define DEFAULT_COMMAND_TERMINATORS           {COMMAND_PROMPT_STRING, \
                                               COMMAND_PROMPT_ERROR_STRING}

/* command strings */
#define HELP_COMMAND_STRING                   "?"
#define SIZE_COMMAND_STRING                   "size "
#define APPEND_COMMAND_STRING                 "append "
#define RESET_COMMAND_STRING                  "reset"

/** string to search for in help output to detect OpenLog */
#define HELP_RESPONSE_SEARCH_STRING           "OpenLog"

/** also absorbs async transmit time */
#define COMMAND_ECHO_TIMEOUT_ms               100
#define RESPONSE_TIMEOUT_ms                   100
#define HELP_RESPONSE_TIMEOUT_ms              500
#define RECEIVE_DELAY_ms                      1
#define CLAIM_TIMEOUT_ms                      1
#define COMMAND_RETRY_COUNT                   3
#define TERMINATORS_COUNT_MAX                 2

#define FILE_PREFIX_LENGTH                    5
#define FILE_ID_LENGTH                        3
#define FILE_EXTENSION_LENGTH                 3
/** FAT filesystem permits 8.3 filenames */
#define FILE_NAME_LENGTH_MAX                  12

/** response to size command for nonexistent file */
#define FILE_SIZE_INVALID                     -1

#define FILE_ID_NONE                          0
/** first file ID */
#define FILE_ID_START                         1
/** last file ID (at most FILE_ID_LENGTH digits) */
#define FILE_ID_LAST                          999

#define FILE_PREFIX_STRING_DEFAULT            "PIKSI"
#define FILE_EXTENSION_STRING                 "BIN"

static const char OPENLOG_MODULE[] =          "openlog";

typedef struct {
  const char *file_name;
} file_configure_params_t;

typedef struct {
  const char *file_name;
  s32 file_size;
} file_size_params_t;

static u32 file_id = FILE_ID_START;
static char file_prefix[FILE_PREFIX_LENGTH+1] = FILE_PREFIX_STRING_DEFAULT;
static usart_dma_state *openlog_usart = 0;

/* USART interface functions */
static bool claim(usart_dma_state *s);
static void release(usart_dma_state *s);

static bool usart_send_str(usart_dma_state *s, const char *str);
static bool usart_issue_command(usart_dma_state *s, const char *str,
                                u32 wait_max_ms);
static bool usart_wait_for_response(usart_dma_state *s, char *response,
                                    u32 response_size, const char **terminators,
                                    u32 terminators_count, u32 wait_max_ms);
static bool usart_wait_for_str(usart_dma_state *s, const char *str,
                               u32 wait_max_ms);
static bool usart_wait_for_strs(usart_dma_state *s, const char **strs,
                                u32 strs_count, u32 wait_max_ms);

/* misc command utility functions */
static bool command_execute(usart_dma_state *s,
                            bool (*func)(usart_dma_state *s, void *params),
                            void *params);
static bool strs_concat(char *dest, u32 dest_len,
                        const char **srcs, u32 srcs_count);
static bool parse_int(s32 *output, const char *str);
static bool terminators_match(char c, const char **terminators,
                              u32 *terminator_indexes, u32 terminators_count);

/* commands - USART must be claimed prior to calling these functions */
static bool command_mode_enter(usart_dma_state *s, void *params);
static bool openlog_discover(usart_dma_state *s, void *params);
static bool openlog_reset(usart_dma_state *s, void *params);
static bool file_size_get(usart_dma_state *s, void *params);
/* note: OpenLog will be in data mode if this function returns success */
static bool file_append_set(usart_dma_state *s, void *params);

static bool log_file_update();
static bool file_prefix_setting_notify(struct setting *setting, const char *val);

static bool claim(usart_dma_state *s)
{
  if(usart_claim_timeout(s, OPENLOG_MODULE, MS2ST(CLAIM_TIMEOUT_ms))) {
    return true;
  } else {
    log_warn("OpenLog - error claiming UART\n");
    return false;
  }
}

static void release(usart_dma_state *s)
{
  usart_release(s);
}

static bool usart_send_str(usart_dma_state *s, const char *str)
{
  u32 len = strlen(str);
  u32 sent = usart_write_dma(&s->tx, (const u8 *)str, len);
  return (sent == len);
}

/** Send a command string and wait for the echo to be received
*
* \param s                  USART to send/receive on
* \param cmd                Command string to send
* \param wait_max_ms        Maximum time to listen before timing out
* \return                   true if the echo was received, otherwise false
*/
static bool usart_issue_command(usart_dma_state *s, const char *cmd,
                                u32 wait_max_ms)
{
  if (!usart_send_str(s, cmd)) {
    return false;
  }

  if (!usart_send_str(s, RETURN_STRING)) {
    return false;
  }

  /* wait for echo */
  if (!usart_wait_for_str(s, cmd, wait_max_ms)) {
    return false;
  }

  return true;
}

/** Wait for a terminator string to be received via the UART
* or until wait_max_ms passes
*
* \param s                  USART to receive from
* \param response           Buffer used to store received data
* \param response_size      Capacity of response buffer
* \param terminators        Array of terminator strings to listen for
* \param terminators_count  Number of terminators provided,
*                           up to TERMINATORS_COUNT_MAX
* \param wait_max_ms        Maximum time to listen before timing out
* \return                   true if a terminator was received, otherwise false
*/
static bool usart_wait_for_response(usart_dma_state *s, char *response,
                                    u32 response_size, const char **terminators,
                                    u32 terminators_count, u32 wait_max_ms)
{
  u32 i;
  u32 terminator_indexes[TERMINATORS_COUNT_MAX];
  u32 response_index = 0;
  systime_t start_ticks = chTimeNow();

  for (i=0; i<terminators_count; i++) {
    terminator_indexes[i] = 0;
  }

  bool timeout = false;
  do {
    /* try to read data */
    u8 c;
    bool data_read = (usart_read_dma(&s->rx, &c, 1) > 0);

    if (data_read) {
      if (response_index < response_size) {
        response[response_index++] = c;
      }

      if (terminators_match(c, terminators,
                            terminator_indexes, terminators_count)) {
        /* null-terminate response (note response_size may be zero) */
        if (response_index+1 < response_size) {
          response_index++;
        }
        if (response_index < response_size) {
          response[response_index] = 0;
        }
        return true;
      }
    }

    timeout = (chTimeNow() - start_ticks) >= MS2ST(wait_max_ms);
    if (timeout) {
      return false;
    }

    if (!data_read) {
      chThdSleepMilliseconds(RECEIVE_DELAY_ms);
    }
  } while (!timeout);

  return false;
}

static bool usart_wait_for_str(usart_dma_state *s, const char *str,
                               u32 wait_max_ms)
{
  return usart_wait_for_response(s, 0, 0, &str, 1, wait_max_ms);
}

static bool usart_wait_for_strs(usart_dma_state *s, const char **strs,
                                u32 strs_count, u32 wait_max_ms)
{
  return usart_wait_for_response(s, 0, 0, strs, strs_count, wait_max_ms);
}

static bool command_execute(usart_dma_state *s,
                            bool (*func)(usart_dma_state *s, void *params),
                            void *params)
{
  u32 retry = 0;
  bool success = false;

  do {
      success = func(s, params);
  } while (!success && (++retry < COMMAND_RETRY_COUNT));

  return success;
}

static bool strs_concat(char *dest, u32 dest_len,
                        const char **srcs, u32 srcs_count)
{
  u32 dest_index = 0;
  u32 srcs_index;

  for (srcs_index = 0; srcs_index < srcs_count; srcs_index++) {
    const char *src = srcs[srcs_index];

    u32 i = 0;
    while (src[i] != 0) {
      if (dest_index < dest_len) {
        dest[dest_index++] = src[i++];
      } else {
        return false;
      }
    }
  }

  /* null terminate */
  if (dest_index < dest_len) {
    dest[dest_index++] = 0;
    return true;
  } else {
    return false;
  }
}

static bool parse_int(s32 *output, const char *str)
{
  u32 str_index = 0;
  bool started = false;
  u32 result = 0;
  bool negative = false;

  while (str[str_index] != 0) {
    char c = str[str_index];

    if (!started && (c == '-')) {
      negative = true;
      started = true;
    } else if ((c >= '0') && (c <= '9')) {
      started = true;
      result = 10 * result + c - '0';
    } else if (started) {
      break;
    }
    str_index++;
  }

  *output = (negative ? -1 : 1) * result;
  return started;
}

/** Incrementally check for multiple terminator string matches
*
* \param c                  Character to process
* \param terminators        Array of terminator strings to match
* \param terminator_indexes Array of indexes into terminator strings. Current
*                           number of matched characters for each terminator
* \param terminators_count  Number of terminators provided
* \return                   true if a terminator was matched, otherwise false
*/
static bool terminators_match(char c, const char **terminators,
                              u32 *terminator_indexes, u32 terminators_count)
{
  u32 i;
  for (i=0; i<terminators_count; i++) {
    /* advance terminator index on match */
    if (c == terminators[i][terminator_indexes[i]]) {
      terminator_indexes[i]++;

      /* return success if the entire terminator has been matched */
      if (terminators[i][terminator_indexes[i]] == 0) {
        return true;
      }
    } else {
      /* reset terminator index on mismatch */
      terminator_indexes[i] = 0;
    }
  }

  return false;
}

static bool command_mode_enter(usart_dma_state *s, void *params)
{
  (void)params;

  const char *terminators[2] = DEFAULT_COMMAND_TERMINATORS;

  /* enter command mode via escape sequence
   * (assuming OpenLog is in data mode) */
  usart_send_str(s, ESCAPE_SEQUENCE_STRING);
  if (usart_wait_for_strs(s, terminators, 2, RESPONSE_TIMEOUT_ms)) {
    return true;
  }

  /* send return to get a fresh prompt
   * (assuming OpenLog is already in command mode) */
  usart_send_str(s, RETURN_STRING);
  if (usart_wait_for_strs(s, terminators, 2, RESPONSE_TIMEOUT_ms)) {
    return true;
  }

  return false;
}

static bool openlog_discover(usart_dma_state *s, void *params)
{
  (void)params;

  /* issue help command */
  if (!usart_issue_command(s, HELP_COMMAND_STRING, COMMAND_ECHO_TIMEOUT_ms)) {
    return false;
  }

  /* check for expected response */
  bool success = usart_wait_for_str(s, HELP_RESPONSE_SEARCH_STRING,
                                    RESPONSE_TIMEOUT_ms);

  /* wait for command to complete */
  const char *terminators[2] = DEFAULT_COMMAND_TERMINATORS;
  /* allow extra time for long help command response */
  usart_wait_for_strs(s, terminators, 2, HELP_RESPONSE_TIMEOUT_ms);

  return success;
}

static bool openlog_reset(usart_dma_state *s, void *params)
{
  (void)params;

  usart_send_str(s, RETURN_STRING RESET_COMMAND_STRING RETURN_STRING);
  return true;
}

static bool file_size_get(usart_dma_state *s, void *params)
{
  const char *file_name = ((file_size_params_t *)params)->file_name;
  s32 *file_size = &((file_size_params_t *)params)->file_size;

  char cmd[32];
  const char *strs[2] = {SIZE_COMMAND_STRING, file_name};
  if (!strs_concat(cmd, sizeof(cmd), strs, 2)) {
    return false;
  }

  if (!usart_issue_command(s, cmd, COMMAND_ECHO_TIMEOUT_ms)) {
    return false;
  }

  char resp[32];
  const char *terminators[2] = DEFAULT_COMMAND_TERMINATORS;
  if (!usart_wait_for_response(s, resp, sizeof(resp),
                               terminators, 2, RESPONSE_TIMEOUT_ms)) {
    return false;
  }

  if (!parse_int(file_size, resp)) {
    return false;
  }

  return true;
}

static bool file_append_set(usart_dma_state *s, void *params)
{
  const char *file_name = ((file_configure_params_t *)params)->file_name;

  char cmd[32];
  const char *strs[2] = {APPEND_COMMAND_STRING, file_name};
  if (!strs_concat(cmd, sizeof(cmd), strs, 2)) {
    return false;
  }

  if (!usart_issue_command(s, cmd, COMMAND_ECHO_TIMEOUT_ms)) {
    return false;
  }

  if (usart_wait_for_str(s, DATA_PROMPT_STRING, RESPONSE_TIMEOUT_ms)) {
    return true;
  } else {
    /* try to re-enter command mode in case the append command went through
     * and OpenLog is now in data mode */
    command_mode_enter(s, 0);
    return false;
  }
}

static bool log_file_update(usart_dma_state *s)
{
  if ((file_id < FILE_ID_START) || (file_id > FILE_ID_LAST)) {
    return false;
  }

  /* enter command mode */
  if (!command_execute(s, command_mode_enter, 0)) {
    log_warn("OpenLog - error entering command mode - resetting\n");
    command_execute(s, openlog_reset, 0);
    return false;
  }

  /* determine next available file ID */
  char file_name[FILE_NAME_LENGTH_MAX+1] = "";
  bool success = false;
  do {
    snprintf(file_name, sizeof(file_name),
             "%s%0" xstr(FILE_ID_LENGTH) "u." FILE_EXTENSION_STRING,
             file_prefix, (unsigned int)file_id);

    file_size_params_t file_size_params =
    {.file_name = file_name, .file_size = 0};
    if (!command_execute(s, file_size_get, &file_size_params)) {
      log_warn("OpenLog - error listing files - resetting\n");
      command_execute(s, openlog_reset, 0);
      return false;
    }

    if (file_size_params.file_size != FILE_SIZE_INVALID) {
      /* file already exists - go to next file ID */
      file_id++;

      if (file_id > FILE_ID_LAST) {
        log_warn("OpenLog - out of files - resetting\n");
        command_execute(s, openlog_reset, 0);
        return false;
      }
    } else {
      success = true;
    }
  } while(!success);

  file_configure_params_t file_configure_params = {.file_name = file_name};
  if (!command_execute(s, file_append_set, &file_configure_params)) {
    log_warn("OpenLog - error configuring log file %s - resetting\n", file_name);
    command_execute(s, openlog_reset, 0);
    return false;
  } else {
    log_info("OpenLog - logging to %s\n", file_name);
    return true;
  }
}

static bool file_prefix_setting_notify(struct setting *setting, const char *val)
{
  /* make sure prefix length is nonzero */
  if (val[0] == 0) {
    return false;
  }

  if (setting->type->from_string(setting->type->priv, setting->addr,
                                 setting->len, val)) {
    /* make sure string is null terminated */
    file_prefix[FILE_PREFIX_LENGTH] = 0;

    /* reset file ID */
    file_id = FILE_ID_START;

    /* TODO: optionally only enact new settings at boot or defer
     * processing for performance reasons */
    if ((openlog_usart != 0) && claim(openlog_usart)) {
      log_file_update(openlog_usart);
      release(openlog_usart);
    }

    return true;
  }

  return false;
}

/** Set up the OpenLog driver */
void openlog_setup()
{
  static struct setting openlog_file_prefix_setting =
  {"openlog", "file_prefix", &file_prefix, FILE_PREFIX_LENGTH,
   file_prefix_setting_notify, NULL, NULL, false};
  settings_register(&openlog_file_prefix_setting, TYPE_STRING);
}

/** Hook called when the UART is configured.
 *  Used to discover a connected OpenLog */
void openlog_configure_hook(usart_dma_state *s, char* uart_name)
{
  if (openlog_usart != 0) {
    /* already discovered an OpenLog */
    return;
  }

  if (!claim(s)) {
    return;
  }

  /* enter command mode and determine if OpenLog is present */
  if (!command_execute(s, command_mode_enter, 0) ||
      !command_execute(s, openlog_discover, 0)) {
    log_info("No OpenLog found on %s\n", uart_name);
  } else {
    log_info("OpenLog found on %s\n", uart_name);
    openlog_usart = s;
    log_file_update(s);
  }

  release(s);
}

/** Start logging to a new file */
void openlog_file_break()
{
  /* TODO: thread safety and/or deferred processing */
  if ((openlog_usart != 0) && claim(openlog_usart)) {
    log_file_update(openlog_usart);
    release(openlog_usart);
  }
}

/** \} */

/** \} */
