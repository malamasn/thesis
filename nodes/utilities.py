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


    @staticmethod
    def closestObstacleBrushfireCffi(start, ogm):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        brushfire[start] = 2

        x = [np.array(v, dtype='int32') for v in brushfire]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        br_c = lib.closestObstacleBrushfire(xi, len(x), len(x[0]))
        brushfire[:] = np.array(x)
        index = np.where(brushfire == -2)
        obstacles = zip(index[0], index[1])

        neighbor_obstacles = []
        final_obstacles = []
        visited = []
        # Cluster obstacle points to neighborhoods
        while len(visited) < len(obstacles):
            for i in obstacles:
                if i not in visited:
                    first = i
                    break

            temp_neighbor_obstacles = [first]
            current = [first]
            next = []
            while current != []:
                for x,y in current:
                    for i in range(-5,6):
                        for j in range(-5,6):
                            xx = x + i
                            yy = y + j
                            if (xx,yy) in obstacles and (xx,yy) not in visited:
                                temp_neighbor_obstacles.append((xx,yy))
                                visited.append((xx,yy))
                                next.append((xx,yy))
                current = next
                next = []
            neighbor_obstacles.append(temp_neighbor_obstacles)

        # Calculate distances for each cluster and return closest to start
        for i in range(len(neighbor_obstacles)):
            obstacle_array = np.array(neighbor_obstacles[i])
            start_array = np.array(start)
            distances = np.linalg.norm(obstacle_array-start_array, axis=1)
            index = distances.argmin()
            min = neighbor_obstacles[i][index]
            final_obstacles.append(min)

        return final_obstacles
