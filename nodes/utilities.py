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
        brushfire[0:,0:] = np.array(y)
        return brushfire

    @staticmethod
    def gvdNeighborBrushfireCffi(start, nodes, gvd):

        brushfire = np.zeros(gvd.shape, np.dtype('int32'))
        brushfire[gvd == 1] = 1
        i,j = zip(*nodes)
        index = (np.array(i), np.array(j))
        brushfire[index] = -1
        brushfire[start] = 2
        brushfire_x = [np.array(v, dtype='int32') for v in brushfire]
        brushfire_c = ffi.new(("int* [%d]") % (len(brushfire_x)))
        for i in range(len(brushfire_x)):
            brushfire_c[i] = ffi.cast("int *", brushfire_x[i].ctypes.data)

        neighbor_nodes = []
        br_c = lib.gvdNeighborBrushfire(brushfire_c, len(brushfire_x), len(brushfire_x[0]))
        brushfire[:] = np.array(brushfire_x)
        index = np.where(brushfire == -2)
        neighbor_nodes = zip(index[0], index[1])
        return neighbor_nodes

    @staticmethod
    def gvdNeighborSplitBrushfireCffi(start, nodes, gvd):

        brushfire = np.zeros(gvd.shape, np.dtype('int32'))
        brushfire[gvd == 1] = 1
        i,j = zip(*nodes)
        index = (np.array(i), np.array(j))
        brushfire[index] = -1
        brushfire[start] = 2

        nn = []
        x = start[0]
        y = start[1]
        for i in range(-1,2):
            for j in range(-1,2):
                if i == 0 and j == 0:
                    continue
                if gvd[x+i,y+j] == 1:
                    nn.append((x+i,y+j))

        brushfire_split = brushfire.copy()
        brushfire_split[nn[0]] = 3
        brushfire_x_1 = [np.array(v, dtype='int32') for v in brushfire_split]
        brushfire_c_1 = ffi.new(("int* [%d]") % (len(brushfire_x_1)))
        for i in range(len(brushfire_x_1)):
            brushfire_c_1[i] = ffi.cast("int *", brushfire_x_1[i].ctypes.data)

        brushfire_split = brushfire.copy()
        brushfire_split[nn[1]] = 3
        brushfire_x_2 = [np.array(v, dtype='int32') for v in brushfire_split]
        brushfire_c_2 = ffi.new(("int* [%d]") % (len(brushfire_x_2)))
        for i in range(len(brushfire_x_2)):
            brushfire_c_2[i] = ffi.cast("int *", brushfire_x_2[i].ctypes.data)

        br_c = lib.gvdNeighborSplitBrushfire(brushfire_c_1, brushfire_c_2, len(brushfire_x_1), len(brushfire_x_1[0]))

        br_1 = np.zeros(gvd.shape)
        br_1[:] = np.array(brushfire_x_1)
        br_2 = np.zeros(gvd.shape)
        br_2[:] = np.array(brushfire_x_2)

        neighbor_nodes = []
        index = np.where(br_1 == -2)
        temp = zip(index[0], index[1])
        neighbor_nodes.append(temp)
        index = np.where(br_2 == -2)
        temp = zip(index[0], index[1])
        neighbor_nodes.append(temp)
        return neighbor_nodes
