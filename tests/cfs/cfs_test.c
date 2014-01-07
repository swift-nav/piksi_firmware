/*
 * Copyright (C) 2013 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
  init(1);

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


