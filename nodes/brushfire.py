#!/usr/bin/env python
import rospy
import numpy as np
import math
from collections import deque

# Class with brushfire functions
class Brushfire:
    # Brushfire starting from obstacles
    def obstacleBrushfire(self, ogm):
        width = ogm.shape[0]
        height = ogm.shape[1]

        # Free space == 0, obstacles == 1, unknown == -1 at brushfire field
        brushfire = np.zeros(ogm.shape, np.dtype('int8'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1

        rospy.loginfo("Brushfire initialized.")
        # Queue obstacles' neighbors
        index = np.where(brushfire == 1)
        queue = zip(index[0], index[1])

        brush_value = 1
        expand = True
        # For all zero pixels do
        while expand:

            expand = False
            brush_value += 1
            next_queue = []
            for pixel in queue:
                x, y = pixel
                for i in range(-1,2):
                    for j in range(-1,2):
                        # Boundary check
                        if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1:
                            continue
                        if i == 0 and j == 0:
                            continue
                        if brushfire[x+i][y+j] == 0:
                            next_queue.append((x+i,y+j))
                            brushfire[x+i][y+j] = brush_value
                            expand = True
            # Keep every value only once
            queue = list(set(next_queue))

        rospy.loginfo("Brushfire done!")
        return brushfire

    # Brushfire on the gvd starting from start node and return indeces of
    # its neighbor nodes
    def gvdNeighborBrushfire(self, start, nodes, gvd):
        width = gvd.shape[0]
        height = gvd.shape[1]

        brushfire = gvd.copy()
        neighbor_nodes = []
        current = []
        current.append(start)
        next = []

        brush_value = 1
        while current != []:
            brush_value += 1
            for pixel in current:
                x,y = pixel
                for i in range(-1,2):
                    for j in range(-1,2):
                        # Boundary check
                        if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1:
                            continue
                        if i == 0 and j == 0:
                            continue
                        # Check if it has been visited
                        if brushfire[x+i,y+j] == 1:
                            if (x+i,y+j) in nodes:
                                neighbor_nodes.append((x+i,y+j))
                            else:
                                next.append((x+i,y+j))
                            brushfire[x+i,y+j] = brush_value
            current = next
            next = []

        return neighbor_nodes
