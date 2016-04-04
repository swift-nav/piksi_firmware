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
  return true;
}

int ndb_fs_open(const char *name, int flags)
{
  return cfs_open(name, flags);
}

void ndb_fs_close(int fd)
{
  cfs_close(fd);
}

cfs_offset_t ndb_fs_seek(int fd, cfs_offset_t offset, int whence)
{
  return cfs_seek(fd, offset, whence);
}

int ndb_fs_remove(const char *name)
{
  return cfs_remove(name);
}

int ndb_fs_read(int fd, void *buf, unsigned size)
{
  return cfs_read(fd, buf, size);
}

int ndb_fs_write(int fd, const void *buf, unsigned size)
{
  return cfs_write(fd, buf, size);
}

int ndb_fs_reserve(const char *name, cfs_offset_t size)
{
  return cfs_coffee_reserve(name, size);
}
