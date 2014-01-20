#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import re
import os

header_file = os.path.join(
  os.path.dirname(__file__),
  '..', 'src', 'sbp_messages.h'
)

with open(header_file, 'r') as f:
  sbph = f.read()

msgs = re.findall('^\s*#define\s+MSG_([A-Z0-9_]+)\s+0x([A-F0-9][A-F0-9])',
                  sbph, re.MULTILINE)

for name, id in msgs:
  globals()[name] = int(id, 16)

