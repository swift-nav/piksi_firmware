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

from mayavi import mlab
import math
import numpy as np
import data_real4 as d

#x_min = -600.0
#x_max = -400.0

#y_min = 0.0
#y_max = 1023.0

x_min = 0.0
x_max = 1000.0

y_min = 0.0
y_max = 1000.0

d.zs = np.array(d.zs)

d.zs = 500 * d.zs / np.max(d.zs)

print d.zs.shape

x_len, y_len = d.zs.shape
xs = np.arange(x_min, x_max, (x_max-x_min)/x_len)
ys = np.arange(y_min, y_max, (y_max-y_min)/y_len)
xs, ys = np.meshgrid(ys, xs)

s = mlab.surf(xs, ys, d.zs)
mlab.show()
