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
        print(len(queue))   # DEBUG
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
                        if x+i < 0 or x+i > width or y+j < 0 or y+j > height:
                            continue
                        if i == 0 and j == 0:
                            continue
                        if brushfire[x+i][y+j] == 0:
                            next_queue.append((x+i,y+j))
                            brushfire[x+i][y+i] = brush_value
                            expand = True
            queue = next_queue
            print(len(queue), brush_value)  # DEBUG

        rospy.loginfo("Brushfire done!")
        return brushfire
