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

        # neighbor_nodes = ffi.new("int **")
        neighbor_nodes = []
        br_c = lib.gvdNeighborBrushfire(brushfire_c, len(brushfire_x), len(brushfire_x[0]))
        for node in nodes:
            i = node[0]
            j = node[1]
            if brushfire_c[i][j] == -2:
                neighbor_nodes.append((i,j))
        neighbor_nodes = list(set(neighbor_nodes))
        print('start', start, 'nn', neighbor_nodes)
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
        brushfire_x = [np.array(v, dtype='int32') for v in brushfire_split]
        brushfire_c_1 = ffi.new(("int* [%d]") % (len(brushfire_x)))
        for i in range(len(brushfire_x)):
            brushfire_c_1[i] = ffi.cast("int *", brushfire_x[i].ctypes.data)
        print('min 1 before', np.min(brushfire_split), 'max_1 before', np.max(brushfire_split))

        brushfire_split = brushfire.copy()
        brushfire_split[nn[1]] = 3
        brushfire_x = [np.array(v, dtype='int32') for v in brushfire_split]
        brushfire_c_2 = ffi.new(("int* [%d]") % (len(brushfire_x)))
        for i in range(len(brushfire_x)):
            brushfire_c_2[i] = ffi.cast("int *", brushfire_x[i].ctypes.data)
        print('min 2 before', np.min(brushfire_split), 'max_2 before', np.max(brushfire_split))
        # neighbor_nodes_first = ffi.new("int **")
        # neighbor_nodes_second = ffi.new("int **")

        br_c = lib.gvdNeighborSplitBrushfire(brushfire_c_1, brushfire_c_2, len(brushfire_x), len(brushfire_x[0]))
        # print('nn_first', neighbor_nodes_first, 'nn_second', neighbor_nodes_second)
        # neighbor_nodes = np.array([neighbor_nodes_first], [neighbor_nodes_second])
        br_1 = np.zeros(gvd.shape)
        for i in range(len(brushfire_x)):
            for j in range(len(brushfire_x[0])):
                br_1[i][j] = brushfire_c_1[i][j]
        br_2 = np.zeros(gvd.shape)
        for i in range(len(brushfire_x)):
            for j in range(len(brushfire_x[0])):
                br_2[i][j] = brushfire_c_2[i][j]
        print('min 1', np.min(br_1), 'max_1', np.max(br_1), 'min 2', np.min(br_2), 'max_2', np.max(br_2))
        neighbor_nodes = []
        temp = []
        for node in nodes:
            i = node[0]
            j = node[1]
            if brushfire_c_1[i][j] == -2:
                temp.append((i,j))
        temp = list(set(temp))
        neighbor_nodes.append(temp)
        temp = []
        for node in nodes:
            i = node[0]
            j = node[1]
            if brushfire_c_2[i][j] == -2:
                temp.append((i,j))
        temp = list(set(temp))
        neighbor_nodes.append(temp)
        print('start', start, 'nn', neighbor_nodes)
        return neighbor_nodes
