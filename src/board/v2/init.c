/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

#include <stdlib.h>
#include <string.h>

#include <libsbp/flash.h>
#include <libsbp/sbp.h>

#include <libswiftnav/logging.h>

#include "main.h"
#include "peripherals/leds.h"
#include "peripherals/random.h"
#include "board/v2/m25_flash.h"
#include "board/nap/nap_common.h"
#include "nap/nap_conf.h"
#include "sbp.h"
#include "error.h"

#define REQUIRED_NAP_VERSION_STR ("v0.16")

#define SCB_AIRCR_VECTKEY                       (0x5FA << SCB_AIRCR_VECTKEY_Pos)
#define SCB_AIRCR_SYSRESETREQ                   (1 << 2)

static void nap_version_check(void);
static void nap_auth_check(void);
static s8 compare_version(const char *a, const char *b);

/** Resets the device back into the bootloader. */
static void reset_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  /* Ensure all outstanding memory accesses including buffered writes are
   * completed before reset.
   */
  __asm__("DSB;");
  /* Keep priority group unchanged. */
  SCB->AIRCR = SCB_AIRCR_VECTKEY |
               SCB_AIRCR_PRIGROUP_Msk |
               SCB_AIRCR_SYSRESETREQ;
  __asm__("DSB;");
  /* Wait until reset. */
  while(1);
}

/** Register the reset_callback. */
static void reset_callback_register(void)
{
  static sbp_msg_callbacks_node_t reset_node;

  sbp_register_cbk(
    SBP_MSG_RESET,
    &reset_callback,
    &reset_node
  );
}

#define STM_UNIQUE_ID_ADDR 0x1FFF7A10
/** Callback to read STM32F4's hardcoded unique ID.
 * Sends STM32F4 unique ID (12 bytes) back to host.
 */
static void stm_unique_id_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;

  sbp_send_msg(SBP_MSG_STM_UNIQUE_ID_RESP, 12, (u8*)STM_UNIQUE_ID_ADDR);
}

/** Register callback to read Device's Unique ID. */
static void stm_unique_id_callback_register(void)
{
  static sbp_msg_callbacks_node_t stm_unique_id_node;

  sbp_register_cbk(SBP_MSG_STM_UNIQUE_ID_REQ,
                   &stm_unique_id_callback,
                   &stm_unique_id_node);
}

void pre_init(void)
{
  /* Delay on start-up as some programmers reset the STM twice. */
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  led_setup();

  /* v2 NAP provides external clock input for the STM and
   * must be configured here. */
  nap_setup();
}

void init(void)
{
  fault_handling_setup();
  reset_callback_register();

  nap_version_check();
  nap_auth_check();
  nap_callbacks_setup();

  rng_setup();
  srand(random_int());

  stm_unique_id_callback_register();
}

static void nap_version_check(void)
{
  /* Check we are running a compatible version of the NAP firmware. */
  char nap_version_string[64] = {0};
  nap_conf_rd_version_string(nap_version_string);
  if (compare_version(nap_version_string, REQUIRED_NAP_VERSION_STR) < 0) {
    while (1) {
      log_error("NAP firmware version >= %s required, please update!"
                "(instructions can be found at http://docs.swift-nav.com/)",
                REQUIRED_NAP_VERSION_STR);
      chThdSleepSeconds(2);
    }
  }
}

/* Check NAP authentication status. Block and print error message
 * if authentication has failed. This must be done after the NAP,
 * USARTs, and SBP subsystems are set up, so that SBP messages and
 * be sent and received (it can't go in init() or nap_setup()).
 */
static void nap_auth_check(void)
{
  u8 nhs = nap_hash_status();
  if (nhs != NAP_HASH_MATCH) {
    led_on(LED_GREEN);
    led_off(LED_RED);
    while (1) {
      DO_EVERY(10000000,
        log_error("NAP Verification Failed");
        led_toggle(LED_GREEN);
        led_toggle(LED_RED);
      );
    }
  }
}

s32 serial_number_get(void)
{
  return nap_conf_rd_serial_number();
}

u8 hw_revision_string_get(char *hw_revision_string)
{
  strcpy(hw_revision_string, nap_conf_rd_hw_rev_string());
  return strlen(hw_revision_string);
}

u8 nap_version_string_get(char *nap_version_string)
{
  return nap_conf_rd_version_string(nap_version_string);
}

/** Compare version strings.
 * Compares a version of the form 'vX.Y-Z-'. If the first character of the
 * version is not 'v' then that string will be considered older than any
 * version string starting with 'v'. Two strings neither starting with 'v' will
 * compare equal.
 *
 * \param a First version string
 * \param b Second version string
 * \return `1` if `a > b`, `-1` if `b > a`, `0` if `a == b`
 */
static s8 compare_version(const char *a, const char *b)
{
  if (a[0] != 'v') {
    if (b[0] != 'v') {
      /* Both have old style version strings, no way to compare. */
      return 0;
    } else {
      /* a has an old style version string, so is older. */
      return -1;
    }
  }

  if (b[0] != 'v') {
    /* b has an old style version string, so is older. */
    return 1;
  }

  char buff[5];
  memset(buff, 0, 5);

  /* Skip initial 'v'. */
  a++; b++;

  /* Extract the major version numbers. */
  u8 major_span = strchr(a, '.') - a;
  strncpy(buff, a, major_span);
  u8 major_a = atoi(buff);
  a += major_span + 1;
  memset(buff, 0, 5);

  major_span = strchr(b, '.') - b;
  strncpy(buff, b, major_span);
  u8 major_b = atoi(buff);
  b += major_span + 1;
  memset(buff, 0, 5);

  if (major_a != major_b) {
    return (major_a < major_b) ? -1 : 1;
  }

  u8 commit_a = 0;
  u8 commit_b = 0;
  u8 minor_a, minor_b;

  /* Check if we have a commit number. */
  if (strchr(a, '-')) {
    /* Extract the minor version numbers. */
    u8 minor_span = strchr(a, '-') - a;
    strncpy(buff, a, minor_span);
    minor_a = atoi(buff);
    a += minor_span + 1;
    memset(buff, 0, 5);

    /* Extract the commit numbers. */
    commit_a = atoi(a);
  } else {
    minor_a = atoi(a);
  }

  /* Check if we have a commit number. */
  if (strchr(b, '-')) {
    /* Extract the minor version numbers. */
    u8 minor_span = strchr(b, '-') - b;
    strncpy(buff, b, minor_span);
    minor_b = atoi(buff);
    b += minor_span + 1;

    /* Extract the commit numbers. */
    commit_b = atoi(b);
  } else {
    minor_b = atoi(b);
  }

  if (minor_a != minor_b) {
    return (minor_a < minor_b) ? -1 : 1;
  }

  return (commit_a < commit_b) ? -1 : (commit_a > commit_b);
}
