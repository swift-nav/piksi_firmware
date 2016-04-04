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

bool ndb_fs_is_real()
{
  return false;
}

int ndb_fs_open(const char *name, int flags)
{
  (void)name;
  (void)flags;
  return 0;
}

void ndb_fs_close(int fd)
{
  (void)fd;
}

cfs_offset_t ndb_fs_seek(int fd, cfs_offset_t offset, int whence)
{
  (void)fd;
  (void)whence;
  assert(whence == CFS_SEEK_SET);
  return offset;
}

int ndb_fs_remove(const char *name)
{
  (void)name;
  return 0;
}

int ndb_fs_read(int fd, void *buf, unsigned size)
{
  (void)fd;
  memset(buf, 0, size);
  return size;
}

int ndb_fs_write(int fd, const void *buf, unsigned size)
{
  (void)fd;
  (void)buf;
  return size;
}

int ndb_fs_reserve(const char *name, cfs_offset_t size)
{
  (void)name;
  (void)size;
  return 0;
}
