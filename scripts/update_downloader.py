#!/usr/bin/env python
# Copyright (C) 2014 Swift Navigation Inc.
# Contact: Colin Beighley <colin@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from urllib2 import urlopen, URLError
from json import load as jsonload
from urlparse import urlparse
import os

INDEX_URL = 'http://downloads.swiftnav.com/index.json'

class UpdateDownloader:

  def __init__(self):
    f = urlopen(INDEX_URL)
    self.index = jsonload(f)
    f.close()

  def download_stm_firmware(self):
    try:
      url = self.index['piksi_v2.3.1']['stm_fw']['url']
      filepath = self._download_file_from_url(url)
    except KeyError:
      raise KeyError("Error downloading firmware: URL not present in index")
    except URLError:
      raise URLError("Error: Failed to download latest NAP firmware from Swift Navigation's website")
    return filepath

  def download_nap_firmware(self):
    try:
      url = self.index['piksi_v2.3.1']['nap_fw']['url']
      filepath = self._download_file_from_url(url)
    except KeyError:
      raise KeyError("Error downloading firmware: URL not present in index")
    except URLError:
      raise URLError("Error: Failed to download latest NAP firmware from Swift Navigation's website")
    return filepath

  def _download_file_from_url(self, url):
    url = url.encode('ascii')
    urlpath = urlparse(url).path
    filename = os.path.split(urlparse(url).path)[1]

    url_file = urlopen(url)
    lines = url_file.readlines()
    with open(filename, 'w') as f:
      for line in lines:
        f.write(line)
    url_file.close()

    return os.path.abspath(filename)
