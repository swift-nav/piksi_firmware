/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
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

#include <libsbp/file_io.h>

#include "sbp.h"
#include "sbp_fileio.h"
#include "sbp_utils.h"
#include "cfs/cfs.h"

static void read_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void read_dir_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void remove_cb(u16 sender_id, u8 len, u8 msg[], void* context);
static void write_cb(u16 sender_id, u8 len, u8 msg[], void* context);

/** Setup file IO
 * Registers relevant SBP callbacks for file IO operations.
 */
void sbp_fileio_setup(void)
{
  static sbp_msg_callbacks_node_t read_node;
  sbp_register_cbk(
    SBP_MSG_FILEIO_READ_REQ,
    &read_cb,
    &read_node
  );
  static sbp_msg_callbacks_node_t read_dir_node;
  sbp_register_cbk(
    SBP_MSG_FILEIO_READ_DIR_REQ,
    &read_dir_cb,
    &read_dir_node
  );
  static sbp_msg_callbacks_node_t remove_node;
  sbp_register_cbk(
    SBP_MSG_FILEIO_REMOVE,
    &remove_cb,
    &remove_node
  );
  static sbp_msg_callbacks_node_t write_node;
  sbp_register_cbk(
    SBP_MSG_FILEIO_WRITE_REQ,
    &write_cb,
    &write_node
  );
}

/** File read callback.
 * Responds to a SBP_MSG_FILEIO_READ_REQ message.
 *
 * Reads a certain length (up to 255 bytes) from a given offset. Returns the
 * data in a SBP_MSG_FILEIO_READ_RESP message where the message length field
 * indicates how many bytes were succesfully read.
 */
static void read_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)context;

  if (sender_id != SBP_SENDER_ID) {
    puts("Invalid sender!");
    return;
  }

  if ((len < 9) || (msg[len-1] != '\0')) {
    puts("Invalid fileio read message!");
    return;
  }

  u32 offset = ((u32)msg[3] << 24) | ((u32)msg[2] << 16) | (msg[1] << 8) | msg[0];
  u8 readlen = MIN(msg[4], SBP_FRAMING_MAX_PAYLOAD_SIZE - len);
  u8 buf[256];
  memcpy(buf, msg, len);
  int f = cfs_open((char*)&msg[5], CFS_READ);
  cfs_seek(f, offset, CFS_SEEK_SET);
  len += cfs_read(f, buf + len, readlen);
  cfs_close(f);

  sbp_send_msg(SBP_MSG_FILEIO_READ_RESP, len, buf);
}

/** Directory listing callback.
 * Responds to a SBP_MSG_FILEIO_READ_DIR_REQ message.
 *
 * The offset parameter can be used to skip the first n elements of the file
 * list.
 *
 * Returns a SBP_MSG_FILEIO_READ_DIR_RESP message containing the directory
 * listings as a NULL delimited list. The listing is chunked over multiple SBP
 * packets and the end of the list is identified by an entry containing just
 * the character 0xFF.
 */
static void read_dir_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)context;

  if (sender_id != SBP_SENDER_ID) {
    puts("Invalid sender!");
    return;
  }

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

  while ((cfs_readdir(&dir, &dirent) == 0) && (len < SBP_FRAMING_MAX_PAYLOAD_SIZE)) {
    strncpy((char*)buf + len, dirent.name, SBP_FRAMING_MAX_PAYLOAD_SIZE - len);
    len += strlen(dirent.name) + 1;
  }

  if (len < SBP_FRAMING_MAX_PAYLOAD_SIZE)
    buf[len++] = 0xff;

  cfs_closedir(&dir);

  sbp_send_msg(SBP_MSG_FILEIO_READ_DIR_RESP, len, buf);
}

/* Remove file callback.
 * Responds to a SBP_MSG_FILEIO_REMOVE message.
 */
static void remove_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)context;

  if (sender_id != SBP_SENDER_ID) {
    puts("Invalid sender!");
    return;
  }

  if ((len < 2) || (msg[len-1] != '\0')) {
    puts("Invalid fileio remove message!");
    return;
  }

  cfs_remove((char*)msg);
}

/* Write to file callback.
 * Responds to a SBP_MSG_FILEIO_WRITE_REQ message.
 *
 * Writes a certain length (up to 255 bytes) at a given offset. Returns a copy
 * of the original SBP_MSG_FILEIO_WRITE_RESP message to check integrity of
 * the write.
 */
static void write_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)context;

  if (sender_id != SBP_SENDER_ID) {
    puts("Invalid sender!");
    return;
  }

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

  sbp_send_msg(SBP_MSG_FILEIO_WRITE_RESP, headerlen, msg);
}
