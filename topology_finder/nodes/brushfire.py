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
        if start in neighbor_nodes:
            neighbor_nodes.remove(start)

        return neighbor_nodes

    # Brushfire on the gvd starting from start node and return indeces of
    # its neighbor nodes grouped by start's first neighbors
    def gvdNeighborSplitBrushfire(self, start, nodes, gvd):
        width = gvd.shape[0]
        height = gvd.shape[1]

        brushfire = gvd.copy()
        all_neighbors = []
        current = [start]

        # Collect start's neighbors at first
        first = []
        x = current[0][0]
        y = current[0][1]
        for i in range(-1,2):
            for j in range(-1,2):
                # Boundary check
                if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1:
                    continue
                if i == 0 and j == 0:
                    brushfire[x+i,y+j] = 2    # To be ignored in next iterations
                    continue
                # Check if it has been visited only on gvd
                if brushfire[x+i,y+j] == 1:
                    first.append((x+i,y+j))
                    brushfire[x+i,y+j] = 3

        while first != []:
            current = []
            current.append(first.pop())
            neighbors = []
            brush_value = 3
            next = []

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
                            # Check if it has been visited only on gvd
                            if brushfire[x+i,y+j] == 1:
                                if (x+i,y+j) in nodes:
                                    neighbors.append((x+i,y+j))
                                else:
                                    next.append((x+i,y+j))
                                brushfire[x+i,y+j] = brush_value
                current = next
                next = []

            all_neighbors.append(neighbors)

        return all_neighbors

    # Brushfire from start point to find closest (one at each obstacle) points
    def closestObstacleBrushfire(self, start, ogm):
        brushfire = np.zeros(ogm.shape, np.dtype('int8'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1

        obstacles = []
        final_obstacles = []

        current = [start]
        next = []
        brush_value = 2
        brushfire[start] = brush_value
        found = False
        last = False
        # Brushfire until it hits an obstacle
        # Then do one more iteration and continue
        while current != [] and not last:
            brush_value += 1
            if found:
                last = True
            for x,y in current:
                for k in range(-1,2):
                    for l in range(-1,2):
                        xx = x + k
                        yy = y + l
                        if brushfire[xx,yy] == 0:
                            brushfire[xx,yy] = brush_value
                            next.append((xx,yy))
                        elif brushfire[xx,yy] == 1:
                            obstacles.append((xx,yy))
                            found = True
            # Keep every value only once
            current = list(set(next))
            next = []
        obstacles = list(set(obstacles))

        neighbor_obstacles = []
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

    # Brushfire from start point until a gvd point is reached, then it is returned
    def pointToGvdBrushfire(self, start, ogm, gvd):
        width = ogm.shape[0]
        height = ogm.shape[1]

        # Free space == 0, obstacles == 1, unknown == -1 at brushfire field
        brushfire = np.zeros(ogm.shape, np.dtype('int8'))
        brushfire[ogm > 49] = 1
        brushfire[ogm == -1] = -1

        queue = [start]

        brush_value = 1
        expand = True
        found = False
        # For all zero pixels do
        while expand and not found:

            expand = False
            brush_value += 1
            next_queue = []
            for pixel in queue:
                x, y = pixel
                if found:
                    break
                for i in range(-1,2):
                    if found:
                        break
                    for j in range(-1,2):
                        # Boundary check
                        if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1:
                            continue
                        if i == 0 and j == 0:
                            continue
                        if gvd[x+i][y+j] == 1:
                            found = True
                            point = (x+i, y+j)
                            break
                        if brushfire[x+i][y+j] == 0:
                            next_queue.append((x+i,y+j))
                            brushfire[x+i][y+j] = brush_value
                            expand = True
            # Keep every value only once
            queue = list(set(next_queue))

        return point
