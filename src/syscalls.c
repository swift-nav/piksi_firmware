/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>

#include <ch.h>

#include <libswiftnav/common.h>
#include <libswiftnav/logging.h>

#include "error.h"

#define WRAP(rtype, proto, usage)     \
  rtype __wrap_##proto                \
  {                                   \
    reent_lock();                     \
    extern rtype __real_##proto;      \
    rtype ret = __real_##usage;       \
    reent_unlock();                   \
    return ret;                       \
  }

typedef struct {
  mutex_t mutex;
  thread_t *owner;
  u8 nest;
} reent_lock_state_t;

static reent_lock_state_t reent_lock_state = {
  .mutex = _MUTEX_DATA(reent_lock_state.mutex),
  .owner = NULL,
  .nest = 0
};

static void reent_lock(void);
static void reent_unlock(void);

static void reent_lock(void)
{
  thread_t *thread = chThdGetSelfX();

  chSysLock();
  if (reent_lock_state.owner != thread) {
    chMtxLockS(&reent_lock_state.mutex);
    reent_lock_state.owner = thread;
  } else {
    reent_lock_state.nest++;
  }
  chSysUnlock();
}

static void reent_unlock(void)
{
  if (reent_lock_state.nest > 0) {
    reent_lock_state.nest--;
  } else {
    reent_lock_state.owner = NULL;
    chMtxUnlock(&reent_lock_state.mutex);
  }
}

/* Wrap vfprintf() functions exported by vfprintf.o */
WRAP(int, vfprintf(FILE *f, _CONST char *fmt, va_list va),
          vfprintf(f, fmt, va))

WRAP(int, _vfprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va),
          _vfprintf_r(r, f, fmt, va))

/* Wrap vfiprintf() functions exported by vfiprintf.o */
WRAP(int, vfiprintf(FILE *f, _CONST char *fmt, va_list va),
          vfiprintf(f, fmt, va))

WRAP(int, _vfiprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va),
          _vfiprintf_r(r, f, fmt, va))

/* Wrap svfprintf() functions exported by svfprintf.o */
WRAP(int, _svfprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va),
          _svfprintf_r(r, f, fmt, va))

/* Wrap svfiprintf() functions exported by svfiprintf.o */
WRAP(int, _svfiprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va),
          _svfiprintf_r(r, f, fmt, va))

/* Wrap vfscanf() functions exported by vfscanf.o */
WRAP(int, vfscanf(FILE *f, const char *fmt, va_list va),
          vfscanf(f, fmt, va))

WRAP(int, _vfscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          _vfscanf_r(r, f, fmt, va))

WRAP(int, __svfscanf(FILE *f, const char *fmt, va_list va),
          __svfscanf(f, fmt, va))

WRAP(int, __svfscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          __svfscanf_r(r, f, fmt, va))

/* Wrap vfiscanf() functions exported by vfiscanf.o */
WRAP(int, vfiscanf(FILE *f, const char *fmt, va_list va),
          vfiscanf(f, fmt, va))

WRAP(int, _vfiscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          _vfiscanf_r(r, f, fmt, va))

WRAP(int, __svfiscanf(FILE *f, const char *fmt, va_list va),
          __svfiscanf(f, fmt, va))

WRAP(int, __svfiscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          __svfiscanf_r(r, f, fmt, va))

/* wrap svfscanf() functions exported by svfscanf.o */
WRAP(int, __ssvfscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          __ssvfscanf_r(r, f, fmt, va))

/* wrap svfiscanf() functions exported by svfiscanf.o */
WRAP(int, __ssvfiscanf_r(struct _reent *r, FILE *f, const char *fmt, va_list va),
          __ssvfiscanf_r(r, f, fmt, va))

/* Implement sbrk() */
void * _sbrk(int incr)
{
  chDbgCheck(incr >= 0);
  void *p = chCoreAlloc(incr);
  if (p == NULL) {
    log_error("sbrk() failed");
    errno = ENOMEM;
    p = (void *)-1;
  }
  return p;
}

/* sprintf() which bypasses REENT mutex */
int fallback_sprintf(char *str, const char *fmt, ...)
{
  int ret;
  va_list ap;
  FILE f;

  f._flags = __SWR | __SSTR;
  f._bf._base = f._p = (unsigned char *) str;
  f._bf._size = f._w = INT_MAX;
  f._file = -1;  /* No file. */
  va_start (ap, fmt);
  extern int  __real__svfprintf_r(struct _reent *r, FILE *f, const char *fmt, va_list va);
  ret = __real__svfprintf_r(_REENT, &f, fmt, ap);
  va_end (ap);
  *f._p = '\0'; /* terminate the string */
  return ret;
}
