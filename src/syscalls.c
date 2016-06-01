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

#include <ch.h>
#include <chprintf.h>

#include "error.h"

/* Replace core printf() functions exported by vfprintf.o */
int _vfprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va)
{
  (void)r;
  int n = chvsnprintf((char *)f->_p, f->_w, fmt, va);
  f->_p += n;
  return n;
}

int _vfiprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va)
{
  return _vfprintf_r(r, f, fmt, va);
}

int _svfprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va)
{
  return _vfprintf_r(r, f, fmt, va);
}

int _svfiprintf_r(struct _reent *r, FILE *f, _CONST char *fmt, va_list va)
{
  return _vfprintf_r(r, f, fmt, va);
}

int vfprintf(FILE *f, _CONST char *fmt, va_list va)
{
  return _vfprintf_r(_impure_ptr, f, fmt, va);
}

int vfiprintf(FILE *f, _CONST char *fmt, va_list va)
{
  return vfprintf(f, fmt, va);
}

/* Complain if heap is reqested */
void * _sbrk_r (struct _reent *r, ptrdiff_t d)
{
  (void)r;
  (void)d;
  screaming_death("sbrk() called");
  return NULL;
}
