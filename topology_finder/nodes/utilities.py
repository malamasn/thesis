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
    def pointBrushfireCffi(start, ogm):

        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        i,j = zip(*start)
        index = (np.array(i), np.array(j))
        brushfire[index] = 2

        y = [np.array(v, dtype='int32') for v in brushfire]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        br_c = lib.pointBrushfire(yi, len(y), len(y[0]))
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

    @staticmethod
    def inRangeObstacleBrushfireCffi(start, ogm, steps, return_all = False):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        brushfire[start] = 2

        x = [np.array(v, dtype='int32') for v in brushfire]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        br_c = lib.inRangeObstacleBrushfire(xi, len(x), len(x[0]), steps)
        brushfire[:] = np.array(x)
        index = np.where(brushfire == -2)
        obstacles = zip(index[0], index[1])

        if return_all:
            return obstacles

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
                    for i in range(-3,4):
                        for j in range(-3,4):
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

    @staticmethod
    def pointToGvdBrushfireCffi(start, ogm, gvd):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = 1
        brushfire[start] = 2
        brushfire[gvd == 1] = -1

        x = [np.array(v, dtype='int32') for v in brushfire]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        br_c = lib.pointToGvdBrushfire(xi, len(x), len(x[0]))
        brushfire[:] = np.array(x)
        index = np.where(brushfire == -2)
        gvd_nodes = zip(index[0], index[1])
        return gvd_nodes

    @staticmethod
    def pointToPointBrushfireCffi(start, finish, ogm):
        starting_step = 2   # it has to be >= 2
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = 1
        brushfire[start] = starting_step
        brushfire[finish] = -1

        x = [np.array(v, dtype='int32') for v in brushfire]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        br_c = lib.pointToPointBrushfire(xi, len(x), len(x[0]))
        brushfire[:] = np.array(x)

        if brushfire[finish] > 0:
            iter_made = brushfire[finish] - starting_step
        else:
            iter_made = -1
        return iter_made

    @staticmethod
    def rectangularBrushfireCoverageCffi(start, ogm, cover_range, fov, theta, sensor_direction):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        brushfire[start] = 2

        y = [np.array(v, dtype='int32') for v in brushfire]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        br_c = lib.rectangularBrushfireCoverage(yi, len(y), len(y[0]), start[0], start[1], \
                theta, cover_range + 2, fov, sensor_direction)
        brushfire[0:,0:] = np.array(y)

        indexes = zip(*np.where(brushfire > 0))
        return indexes

    @staticmethod
    def circularBrushfireCoverageCffi(start, ogm, cover_range, fov, theta, sensor_direction):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        brushfire[start] = 2

        y = [np.array(v, dtype='int32') for v in brushfire]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        br_c = lib.circularBrushfireCoverage(yi, len(y), len(y[0]), start[0], start[1], \
                theta, cover_range + 2, fov, sensor_direction)
        brushfire[0:,0:] = np.array(y)

        indexes = zip(*np.where(brushfire > 0))
        return indexes

    @staticmethod
    def circularRayCastCoverageCffi(start, ogm, cover_range, fov, theta, sensor_direction, return_obstacles = False):
        brushfire = np.zeros(ogm.shape, np.dtype('int32'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1
        brushfire[start] = 2

        y = [np.array(v, dtype='int32') for v in brushfire]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        br_c = lib.circularRayCastCoverage(yi, len(y), len(y[0]), start[0], start[1], \
                theta, cover_range, fov, sensor_direction)
        brushfire[0:,0:] = np.array(y)

        if return_obstacles:
            indexes = zip(*np.where(brushfire == -2))
        else:
            indexes = zip(*np.where(brushfire > 1))
        return indexes
