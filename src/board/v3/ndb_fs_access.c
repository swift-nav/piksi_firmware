/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>
#include <cfs/cfs-coffee.h>
#include <cfs/cfs.h>

/* #define USE_CFS */

bool ndb_fs_is_real()
{
#ifdef USE_CFS
  return true;
#else
  return false;
#endif
}

int ndb_fs_open(const char *name, int flags)
{
#ifdef USE_CFS
  return cfs_open(name, flags);
#else
  (void)name;
  (void)flags;
  return 0;
#endif
}

void ndb_fs_close(int fd)
{
#ifdef USE_CFS
  cfs_close(fd);
#else
  (void)fd;
#endif
}

cfs_offset_t ndb_fs_seek(int fd, cfs_offset_t offset, int whence)
{
#ifdef USE_CFS
  return cfs_seek(fd, offset, whence);
#else
  (void)fd;
  (void)whence;
  assert(whence == CFS_SEEK_SET);
  return offset;
#endif
}

int ndb_fs_remove(const char *name)
{
#ifdef USE_CFS
  return cfs_remove(name);
#else
  (void)name;
  return 0;
#endif
}

int ndb_fs_read(int fd, void *buf, unsigned size)
{
#ifdef USE_CFS
  return cfs_read(fd, buf, size);
#else
  (void)fd;
  memset(buf, 0, size);
  return size;
#endif
}

int ndb_fs_write(int fd, const void *buf, unsigned size)
{
#ifdef USE_CFS
  return cfs_write(fd, buf, size);
#else
  (void)fd;
  (void)buf;
  return size;
#endif
}

int ndb_fs_reserve(const char *name, cfs_offset_t size)
{
#ifdef USE_CFS
  return cfs_coffee_reserve(name, size);
#else
  (void)name;
  (void)size;
  return 0;
#endif
}
