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
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <libswiftnav/logging.h>
#include "libsbp/piksi.h"
#include "sbp.h"
#include "ndb.h"
#include "ndb_internal.h"

#define NDB_THREAD_PRIORITY (LOWPRIO)
static WORKING_AREA_CCM(ndb_thread_wa, 2048);
static void ndb_service_thread(void*);

u8 ndb_file_version[MAX_NDB_FILE_VERSION_LEN];
u8 ndb_file_end_mark = 0xb6;

static MUTEX_DECL(data_access);

static ndb_element_metadata_t *wq_first = NULL;
static ndb_element_metadata_t *wq_last = NULL;

static enum ndb_op_code ndb_wq_put(ndb_element_metadata_t* md);
static void ndb_wq_get(ndb_element_metadata_t** md);

static enum ndb_op_code ndb_open_file(ndb_file_t *file);
static enum ndb_op_code ndb_read(ndb_file_t *f, cfs_offset_t o,
                                 void *first_element, size_t s);

static void ndb_wq_process(bool *data_write_ok, bool *md_write_ok);
static bool ndb_wq_empty();

cfs_offset_t ndb_fs_seek(int fd, cfs_offset_t offset, int whence);
int ndb_fs_open(const char *name, int flags);
void ndb_fs_close(int fd);
int ndb_fs_remove(const char *name);
int ndb_fs_read(int fd, void *buf, unsigned size);
int ndb_fs_write(int fd, const void *buf, unsigned size);
int ndb_fs_reserve(const char *name, cfs_offset_t size);
bool ndb_fs_is_real();

bool ndb_wq_empty()
{
  bool r = false;
  ndb_lock();
  r = NULL == wq_first;
  if (r)
    assert(NULL == wq_last);
  else
    assert(NULL != wq_last);
  ndb_unlock();
  return r;
}

enum ndb_op_code ndb_wq_put(ndb_element_metadata_t* md)
{
  if (NULL == wq_last) {
    wq_first = wq_last = md;
  } else {
    wq_last->next = md;
    wq_last = md;
  }
  md->next = NULL;
  md->nv_data.state |= NDB_ENQUEUED;
  return NDB_ERR_NONE;
}

void ndb_wq_get(ndb_element_metadata_t** md)
{
  *md = wq_first;
  if (NULL != wq_first) {
    wq_first = wq_first->next;

    if (NULL == wq_first)
      wq_last = NULL;
  }
}


void ndb_init()
{
  if (!ndb_fs_is_real())
    log_info("NDB: configured not to save data to flash file system");

  u16 vlen = sizeof(GIT_VERSION);
  vlen = vlen > sizeof(ndb_file_version) ? sizeof(ndb_file_version) : vlen;
  memcpy(ndb_file_version, GIT_VERSION, vlen);
  if (vlen < sizeof(ndb_file_version)) {
    memset(&ndb_file_version[vlen], 0xb6, sizeof(ndb_file_version) - vlen);
  }
}

void ndb_start()
{
  chThdCreateStatic(ndb_thread_wa, sizeof(ndb_thread_wa),
                    NDB_THREAD_PRIORITY, ndb_service_thread, NULL);
}

void ndb_log_file_open(const enum ndb_op_code oc, const char* file_type,
                   const ndb_file_t* file)
{
  switch (oc) {
    case NDB_ERR_NONE:
      log_info("Opened %s file (%d)", file_type, file->fh);
      break;
    case NDB_ERR_FILE_IO:
      log_error("Cannot open %s file", file_type);
      break;
    case NDB_ERR_INIT_DONE:
      log_info("No %s file present in flash, create an empty one (%d)",
               file_type, file->fh);
      break;
    default:
      assert(!"ndb_log_file_open()");
      break;
  }
}

void ndb_load_data(ndb_file_t *f,
                   const char* ftype,
                   void *data,
                   ndb_element_metadata_t* metadata,
                   const size_t el_size,
                   const size_t el_number)
{
  enum ndb_op_code r = NDB_ERR_FILE_IO;
  size_t ds = el_size * el_number;
  size_t mds = sizeof(ndb_element_metadata_nv_t) * el_number;
  ndb_element_metadata_nv_t md_nv[el_number];

  if (ndb_open_file(f) == NDB_ERR_NONE) {
    if (ndb_read(f, 0, data, ds) == NDB_ERR_NONE) {
      if (ndb_read(f, ds, md_nv, mds) == NDB_ERR_NONE) {
        u32 i;
        for (i = 0; i < el_number; i++) {
          metadata[i].nv_data = md_nv[i];
        }
        r = NDB_ERR_NONE;
      }
    }
  }

  if (NDB_ERR_NONE != r) {
    /* Initialize data element and metadata to zeros */
    memset(data, 0, ds);
    memset(metadata, 0, sizeof(ndb_element_metadata_t) * el_number);

    if (f->fh >= 0) {
      /* And save to file */
      memset(md_nv, 0, mds);
      if (ndb_write_file_data(f, 0, data, ds) == NDB_ERR_NONE) {
        if (ndb_write_file_data(f, ds, md_nv, mds) == NDB_ERR_NONE) {
          /* Write non zero byte to the end of file to work around coffee fs
           * feature of dropping zeros from the end of file. */
          if (ndb_write_file_data(f, ds + mds,
                                  &ndb_file_end_mark, 1) == NDB_ERR_NONE) {
            /* Indicate to the level above that initialization was done */
            r = NDB_ERR_INIT_DONE;
          }
        }
      }
    }
  }

  ndb_log_file_open(r, ftype, f);

  for (u32 i = 0; i < el_number; i++) {
    metadata[i].data = data + (i * el_size);
    metadata[i].index = i;
    metadata[i].file = f;
    metadata[i].next = NULL;
    metadata[i].update_c = 1;
  }
}

ndb_timestamp_t ndb_get_timestamp()
{
  return chVTGetSystemTime();
}

static void ndb_service_thread(void* p)
{
  (void) (p);
  chRegSetThreadName("ndb");
  bool data_write_ok = false;
  bool md_write_ok = false;

  chThdSleepMilliseconds(NV_WRITE_REQ_TIMEOUT);
  while (true) {
    chThdSleepMilliseconds(NV_WRITE_REQ_TIMEOUT);
    while(!ndb_wq_empty()) {
      ndb_wq_process(&data_write_ok, &md_write_ok);
      log_debug("Saving to NVM: (%s, %s)", (data_write_ok ? "OK" : "failed"), (md_write_ok ? "OK" : "failed"));
      if (!data_write_ok)
        log_error("Error writing data to the NDB file");
      if (!md_write_ok)
        log_error("Error writing metadata to the NDB file");
    }
    ndb_sbp_updates();
  }
}

void ndb_wq_process(bool* data_write_ok, bool* md_write_ok)
{
  ndb_element_metadata_t* md;

  *data_write_ok = *md_write_ok = false;

  ndb_lock();

  ndb_wq_get(&md);

  assert(md != NULL);

  ndb_element_metadata_t md_copy;

  u16 element_size = md->file->data_size;
  u8 buf[element_size];

  assert((md->nv_data.state & (NDB_IE_DIRTY + NDB_MD_DIRTY)) != 0);
  assert((md->nv_data.state & NDB_ENQUEUED) != 0);

  if(md->nv_data.state & NDB_IE_DIRTY)
    memcpy(&buf, md->data, element_size);
  memcpy(&md_copy, md, sizeof(ndb_element_metadata_t));

  md->nv_data.state &= ~(NDB_IE_DIRTY + NDB_MD_DIRTY + NDB_ENQUEUED);

  ndb_unlock();

  enum ndb_op_code ret;

  if(md_copy.nv_data.state & NDB_IE_DIRTY)
    ret = ndb_write_file_data(md->file, element_size * md->index, &buf,
                              element_size);
  else
    ret = NDB_ERR_NONE;

  *data_write_ok = NDB_ERR_NONE == ret;

  md_copy.nv_data.state &= ~(NDB_IE_DIRTY + NDB_MD_DIRTY + NDB_ENQUEUED);

  u16 n_elements = md->file->n_elements;

  cfs_offset_t offset = element_size * n_elements
      + sizeof(ndb_element_metadata_nv_t) * md->index;
  ret = ndb_write_file_data(md->file, offset, &md_copy.nv_data,
                            sizeof(ndb_element_metadata_nv_t));
  *md_write_ok = NDB_ERR_NONE == ret;
}

int ndb_read_pos(int fd, void *buf, unsigned size, int offset)
{
  int rets = ndb_fs_seek(fd, offset, CFS_SEEK_SET);
  if (rets == offset) {
      return ndb_fs_read(fd, buf, size);
  }
  return rets;
}

int ndb_write_pos(int fd, void *buf, unsigned size, int offset)
{
  int rets = ndb_fs_seek(fd, offset, CFS_SEEK_SET);
  if (rets == offset) {
    return ndb_fs_write(fd, buf, size);
  }
  return rets;
}

bool ndb_file_verion_match(const char* version)
{
  if (ndb_fs_is_real())
    return memcmp(version, ndb_file_version, sizeof(ndb_file_version)) == 0;
  else
    return true;
}

/**
 * Opens NDB file that stores information elements of certain type.
 * This function checks if version of the file matches the passed one
 * and if it doesn't creates empty file automatically.
 */
enum ndb_op_code ndb_open_file(ndb_file_t *file)
{
  char ver[MAX_NDB_FILE_VERSION_LEN];

  file->fh = ndb_fs_open(file->name, CFS_READ);
  if (file->fh < 0) {
    ndb_fs_reserve(file->name,
                       file->expected_size + sizeof(ndb_file_version));
  } else {
    ndb_fs_close(file->fh);
  }

  file->fh = ndb_fs_open(file->name, CFS_READ + CFS_WRITE);
  if (file->fh < 0)
    return NDB_ERR_FILE_IO;

  if (ndb_read_pos(file->fh, ver, sizeof(ndb_file_version), 0)
      == sizeof(ndb_file_version)) {
    if(ndb_file_verion_match(ver)) {
      return NDB_ERR_NONE;
    } else {
      ndb_fs_close(file->fh);
      ndb_fs_remove(file->name);
      ndb_fs_reserve(file->name,
                         file->expected_size + sizeof(ndb_file_version));
      file->fh = ndb_fs_open(file->name, CFS_READ + CFS_WRITE);
      if (file->fh < 0)
        return NDB_ERR_FILE_IO;
    }
  }

  if (ndb_write_pos(file->fh, ndb_file_version, sizeof(ndb_file_version), 0)
      != sizeof(ndb_file_version)) {
    ndb_fs_close(file->fh);
    file->fh = -1;
    return NDB_ERR_FILE_IO;
  }

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_write_file_data(ndb_file_t *f, cfs_offset_t o, void *b,
                                     size_t l)
{
  if (f->fh < 0)
    return NDB_ERR_FILE_IO;

  int offset = sizeof(ndb_file_version) + o;
  int written = ndb_write_pos(f->fh, b, l, offset);
  return (written == (int) l) ? NDB_ERR_NONE : NDB_ERR_FILE_IO;
}

/**
 * Reading from NDB file.
 * This function is to be called only during initialization of NDB.
 * It is intended to read all information elements collection from
 * file into memory.
 */
enum ndb_op_code ndb_read(ndb_file_t *file, cfs_offset_t o,
                          void *first_element, size_t size)
{
  if (file->fh < 0)
    return NDB_ERR_FILE_IO;

  int offset = sizeof(ndb_file_version) + o;
  int read =  ndb_read_pos(file->fh, first_element, size, offset);
  return (read == (int) size) ? NDB_ERR_NONE : NDB_ERR_FILE_IO;
}

void ndb_lock()
{
  chMtxLock(&data_access);
}

void ndb_unlock()
{
  chMtxUnlock(&data_access);
}

void ndb_retrieve(void* out, void* cached, size_t size)
{
  ndb_lock();
  memcpy(out, cached, size);
  ndb_unlock();
}

enum ndb_op_code ndb_update(void* new, enum ndb_data_source src,
                            ndb_element_metadata_t *md)
{
  void* cached = md->data;
  size_t size = md->file->data_size;

  ndb_lock();

  if (md->nv_data.state & NDB_ENQUEUED) {
    ndb_unlock();
    log_info("MD %p is already in the queue", md);
    return NDB_ERR_BUSY;
  }

  md->nv_data.received_at = ndb_get_timestamp();
  md->nv_data.state |= NDB_MD_DIRTY;
  md->nv_data.source = src;

  if (memcmp(new, cached, size) == 0) {
    ndb_wq_put(md);
    ndb_unlock();
    return NDB_ERR_NONE;
  }

  memcpy(cached, new, size);
  md->update_c++;
  if (!md->update_c)
    md->update_c++; /* should never be 0 */
  md->nv_data.state |= NDB_IE_VALID;
  md->nv_data.state |= NDB_IE_DIRTY;
  ndb_wq_put(md);

  ndb_unlock();
  return NDB_ERR_NONE;
}
