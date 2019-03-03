#!/usr/bin/env python
import rospy
import numpy as np
import math

# Class with topological functions
class Topology:
    # Calculate GVD using brushfire
    def gvd(self, ogm, brushfire):
        # Set obstacles and first pixel around them as non gvd pixels
        voronoi = np.full(ogm.shape, -1)
        voronoi[brushfire < 3] = 0

        width = ogm.shape[0]
        height = ogm.shape[1]

        for i in range(width):
            for j in range(height):
                if voronoi[i][j] == 0:
                    continue
        # CHECK ALL -1 IF THEY CAN BE ADDED TO GVD ( = 1) OR NOT ( = 0)

        return voronoi

    # Find useful topological nodes from gvd
    def topologicalNodes(self, ogm, brushfire, gvd):
        nodes = []
        width = ogm.shape[0]
        height = ogm.shape[1]

        # Check all candidate nodes
        index = np.where((gvd[1:width-1][1:height-1]==1) \
                        & (brushfire[1:width-1][1:height-1] > 3))
        for i in range(len(index)):
            x = index[0][i]
            y = index[1][i]

            # Count neighbors at gvd inculding node
            count_neighbors = np.sum(gvd[x-1:x+2][y-1:y+2])

            # Add 1-neighbor and 3-neighbor nodes to graph
            if count_neighbors == 2 or count_neighbors == 4:
                nodes.append([x,y])

            # Add initial 2-neighbor nodes
            elif count_neighbors == 3:
                notGood = False
                diff = False
                min = brushfire[x][y]
                for i in range(-10,10):
                    for j in range(-10,10):
                        # Boundary check
                        if x+i < 0 or x+i > width or y+j < 0 or y+j > height or gvd[x+i][y+j] == 0:
                            continue
                        if brushfire[x+i][y+j] < min:
                            notGood = True
                        if (brushfire[x+i][y+j] + 0.5) != min:
                            diff = True
                if not notGood and diff:
                    nodes.append([x,y])

        # Recheck nodes with 2 neighbors
        for i in range(len(nodes)):
            x = nodes[i][0]
            y = nodes[i][1]

            # Count neighbors at gvd inculding node
            count_neighbors = np.sum(gvd[x-1:x+2][y-1:y+2])

            if count_neighbors != 3:
                continue

            # Find mean of 20x20 neighbor area
            sum = 0
            count = 0
            for i in range(-10,10):
                for j in range(-10,10):
                    # Boundary check
                    if x+i < 0 or x+i > width or y+j < 0 or y+j > height or gvd[x+i][y+j] == 0:
                        continue
                    sum += brushfire[x+i][y+j]
                    counter += 1
            if sum/counter - brushfire[x][y] < 3.5:
                del nodes[i]

        # Check if nodes are close and del some
        ## NOT DONE YET

        return nodes
