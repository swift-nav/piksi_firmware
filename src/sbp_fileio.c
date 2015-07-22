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
#include <alloca.h>

#include <libsbp/file_io.h>
#include <libswiftnav/logging.h>

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
static void read_cb(u16 sender_id, u8 len, u8 msg_[], void* context)
{
  (void)context;
  msg_fileio_read_req_t *msg = (msg_fileio_read_req_t *)msg_;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender!\n");
    return;
  }

  if ((len <= sizeof(*msg)) || (len == SBP_FRAMING_MAX_PAYLOAD_SIZE)) {
    log_error("Invalid fileio read message!\n");
    return;
  }

  /* Add a null termination to filename */
  msg_[len] = 0;

  msg_fileio_read_resp_t *reply;
  u8 readlen = MIN(msg->chunk_size, SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(*reply));
  reply = alloca(sizeof(msg_fileio_read_resp_t) + readlen);
  reply->sequence = msg->sequence;
  int f = cfs_open(msg->filename, CFS_READ);
  cfs_seek(f, msg->offset, CFS_SEEK_SET);
  readlen = cfs_read(f, &reply->contents, readlen);
  cfs_close(f);

  sbp_send_msg(SBP_MSG_FILEIO_READ_RESP,
               sizeof(*reply) + readlen, (u8*)reply);
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
static void read_dir_cb(u16 sender_id, u8 len, u8 msg_[], void* context)
{
  (void)context;
  msg_fileio_read_dir_req_t *msg = (msg_fileio_read_dir_req_t *)msg_;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender!\n");
    return;
  }

  if ((len <= sizeof(*msg)) || (len == SBP_FRAMING_MAX_PAYLOAD_SIZE)) {
    log_error("Invalid fileio read dir message!\n");
    return;
  }

  /* Add a null termination to dirname */
  msg_[len] = 0;

  struct cfs_dir dir;
  struct cfs_dirent dirent;
  u32 offset = msg->offset;
  msg_fileio_read_dir_resp_t *reply = alloca(SBP_FRAMING_MAX_PAYLOAD_SIZE);
  reply->sequence = msg->sequence;
  cfs_opendir(&dir, msg->dirname);
  while (offset && (cfs_readdir(&dir, &dirent) == 0))
    offset--;

  len = 0;
  size_t max_len = SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(*reply);
  while (cfs_readdir(&dir, &dirent) == 0) {
    if (strlen(dirent.name) > (max_len - len - 1))
      break;
    strcpy((char*)reply->contents + len, dirent.name);
    len += strlen(dirent.name) + 1;
  }

  cfs_closedir(&dir);

  sbp_send_msg(SBP_MSG_FILEIO_READ_DIR_RESP,
               sizeof(*reply) + len, (u8*)reply);
}

/* Remove file callback.
 * Responds to a SBP_MSG_FILEIO_REMOVE message.
 */
static void remove_cb(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)context;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender!\n");
    return;
  }

  if ((len < 1) || (len == SBP_FRAMING_MAX_PAYLOAD_SIZE)) {
    log_error("Invalid fileio remove message!\n");
    return;
  }

  /* Add a null termination to filename */
  msg[len] = 0;

  cfs_remove((char*)msg);
}

/* Write to file callback.
 * Responds to a SBP_MSG_FILEIO_WRITE_REQ message.
 *
 * Writes a certain length (up to 255 bytes) at a given offset. Returns a copy
 * of the original SBP_MSG_FILEIO_WRITE_RESP message to check integrity of
 * the write.
 */
static void write_cb(u16 sender_id, u8 len, u8 msg_[], void* context)
{
  (void)context;
  msg_fileio_write_req_t *msg = (msg_fileio_write_req_t *)msg_;

  if (sender_id != SBP_SENDER_ID) {
    log_error("Invalid sender!\n");
    return;
  }

  if ((len <= sizeof(*msg) + 2) ||
      (strnlen(msg->filename, SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(*msg)) ==
                              SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(*msg))) {
    log_error("Invalid fileio write message!\n");
    return;
  }

  u8 headerlen = sizeof(*msg) + strlen(msg->filename) + 1;
  int f = cfs_open(msg->filename, CFS_WRITE);
  cfs_seek(f, msg->offset, CFS_SEEK_SET);
  cfs_write(f, msg_ + headerlen, len - headerlen);
  cfs_close(f);

  msg_fileio_write_resp_t reply = {.sequence = msg->sequence};
  sbp_send_msg(SBP_MSG_FILEIO_WRITE_RESP, sizeof(reply), (u8*)&reply);
}
