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

#include "sbp.h"
#include "sbp_fileio.h"
#include "cfs/cfs.h"

#include <stdio.h>
#include <string.h>

static void read_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void read_dir_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void remove_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void write_cb(u16 sender_id, u8 len, u8 msg[], void* context);

void sbp_fileio_setup(void)
{
  static sbp_msg_callbacks_node_t read_node;
  sbp_register_cbk(
    MSG_FILEIO_READ,
    &read_cb,
    &read_node
  );
  static sbp_msg_callbacks_node_t read_dir_node;
  sbp_register_cbk(
    MSG_FILEIO_READ_DIR,
    &read_dir_cb,
    &read_dir_node
  );
  static sbp_msg_callbacks_node_t remove_node;
  sbp_register_cbk(
    MSG_FILEIO_REMOVE,
    &remove_cb,
    &remove_node
  );
  static sbp_msg_callbacks_node_t write_node;
  sbp_register_cbk(
    MSG_FILEIO_WRITE,
    &write_cb,
    &write_node
  );
}

static void read_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id;
  (void)context;

  if ((len < 9) || (msg[len-1] != '\0')) {
    puts("Invalid fileio read message!");
    return;
  }

  u32 offset = ((u32)msg[3] << 24) | ((u32)msg[2] << 16) | (msg[1] << 8) | msg[0];
  u8 readlen = msg[4];
  u8 buf[256];
  memcpy(buf, msg, len);
  int f = cfs_open((char*)&msg[5], CFS_READ);
  cfs_seek(f, offset, CFS_SEEK_SET);
  len += cfs_read(f, buf + len, readlen);
  cfs_close(f);

  sbp_send_msg(MSG_FILEIO_READ, len, buf);
}

static void read_dir_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id;
  (void)context;

  if ((len < 5) || (msg[len-1] != '\0')) {
    puts("Invalid fileio read dir message!");
    return;
  }

  u32 offset = ((u32)msg[3] << 24) | ((u32)msg[2] << 16) | (msg[1] << 8) | msg[0];
  struct cfs_dir dir;
  struct cfs_dirent dirent;
  u8 buf[256];
  memcpy(buf, msg, len);
  cfs_opendir(&dir, (char*)&msg[4]);
  while (offset && (cfs_readdir(&dir, &dirent) == 0))
    offset--;

  while ((cfs_readdir(&dir, &dirent) == 0) && (len < 255)) {
    strncpy((char*)buf + len, dirent.name, 255 - len);
    len += strlen(dirent.name) + 1;
  }

  if (len < 255)
    buf[len++] = 0xff;

  cfs_closedir(&dir);

  sbp_send_msg(MSG_FILEIO_READ_DIR, len, buf);
}

static void remove_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id;
  (void)context;

  if ((len < 2) || (msg[len-1] != '\0')) {
    puts("Invalid fileio remove message!");
    return;
  }

  cfs_remove((char*)msg);
}

static void write_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id;
  (void)context;

  if (len < 6) {
    puts("Invalid fileio write message!");
    return;
  }

  u32 offset = ((u32)msg[3] << 24) | ((u32)msg[2] << 16) | (msg[1] << 8) | msg[0];
  u8 headerlen = 4 + strlen((char*)&msg[4]) + 1;
  int f = cfs_open((char*)&msg[4], CFS_WRITE);
  cfs_seek(f, offset, CFS_SEEK_SET);
  cfs_write(f, msg + headerlen, len - headerlen);
  cfs_close(f);

  sbp_send_msg(MSG_FILEIO_WRITE, headerlen, msg);
}
