#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from _cpp_functions import ffi, lib

class Cffi:
    @staticmethod
    def obstacleBrushfireCffi(ogm):

        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1

        x = [np.array(v, dtype='int32') for v in ogm]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        y = [np.array(v, dtype='int32') for v in brushfire]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        br_c = lib.obstacleBrushfire(xi, yi, len(x), len(x[0]))
        for i in range(ogm.shape[0]):
            for j in range(ogm.shape[1]):
                brushfire[i][j] = yi[i][j]
        return brushfire
