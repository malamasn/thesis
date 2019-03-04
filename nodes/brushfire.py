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
        # Free space == 0 and obstacles == 1 at brushfire field
        brushfire = np.zeros(ogm.shape, np.dtype('int8'))
        brushfire[np.logical_or((ogm > 49), (ogm == -1))] = 1
        rospy.loginfo("Brushfire initialized.")
        # Queue obstacles' neighbors
        queue = deque()
        for x in range(1, width-1):     # MAKE IT FASTER
            for y in range(1, height-1):
                # If already brushfired skip
                if brushfire[x][y]:
                    continue
                # neighbor checks around ogm[x][y] for obstacles
                neighbor = np.any(brushfire[x-1:x+2][y-1:y+2] == 1)
                if ogm[x][y] < 50 and neighbor:
                    queue.append((x, y))
        print(len(queue))   # DEBUG
        # For all zero pixels do
        while(queue != []):
            x, y = queue.popleft()
            # If already brushfired skip
            if brushfire[x][y]:
                continue
            # Add non brushfired neighbors of pixel[x][y]
            brush_value = np.inf
            for i in range(-1,2):
                for j in range(-1,2):
                    # Boundary check
                    if x+i < 0 or x+i > width or y+j < 0 or y+j > height:
                        continue
                    if i == 0 and j == 0:
                        continue
                    if not brushfire[x+i][y+j]:
                        queue.append((x+i,y+j))
                    else:
                        if brushfire[x+i][y+j] < brush_value:
                            brush_value = brushfire[x+i][y+j]
            # Brushfire value comes from closest obstacle
            brushfire[x][y] = brush_value + 1
        rospy.loginfo("Brushfire done!")
        return brushfire
