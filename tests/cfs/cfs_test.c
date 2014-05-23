/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"
#include "cfs/cfs-coffee-arch.h"
#define COFFEE_SECTOR_COUNT	(unsigned)(COFFEE_SIZE / COFFEE_SECTOR_SIZE)

int main()
{
  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- COFFEE TEST ---\n");

  char buf[80] = "Hello, world\n";

  int ret = cfs_coffee_format();

  if (ret < 0)
    while(1);

  int fd = cfs_open("test", CFS_READ | CFS_WRITE);

  if(fd >= 0) {
    ret = cfs_write(fd, buf, strlen(buf));
    if (ret < 0) while(1);
    ret = cfs_write(fd, "JELLO\n", 7);
    if (ret < 0) while(1);
    ret = cfs_seek(fd, 0, CFS_SEEK_SET);
    if (ret < 0) while(1);
    ret = cfs_read(fd, buf, sizeof(buf));
    if (ret < 0) while(1);
    printf("Read message: %s\n", buf);
    ret = cfs_seek(fd, 0, CFS_SEEK_SET);
    if (ret < 0) while(1);
    ret = cfs_write(fd, "JELLO", 5);
    if (ret < 0) while(1);
    ret = cfs_seek(fd, 0, CFS_SEEK_SET);
    if (ret < 0) while(1);
    ret = cfs_read(fd, buf, sizeof(buf));
    if (ret < 0) while(1);
    printf("Read message: %s\n", buf);
    cfs_close(fd);
    if (ret < 0) while(1);
  }

  while (1);

	return 0;
}


