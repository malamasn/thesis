#!/usr/bin/env python
import rospy
import numpy as np
import math
from skimage.morphology import skeletonize

# Class with topological functions
class Topology:
    # Calculate GVD using brushfire
    def gvd(self, ogm, brushfire):
        # Set obstacles and first pixels around them as non gvd pixels
        voronoi = np.zeros(ogm.shape)
        rospy.loginfo("GVD initialized.")
        # Set free space as starting gvd and skeletonize it to get one pixel diagram
        voronoi[brushfire > 3] = 1
        voronoi = skeletonize(voronoi)

        ## DO I NEED THINNING HERE ?
        rospy.loginfo("GVD done!")
        return voronoi

    # Find useful topological nodes from gvd
    def topologicalNodes(self, ogm, brushfire, gvd):
        nodes = []
        width = ogm.shape[0]
        height = ogm.shape[1]
        rospy.loginfo("Initlizing topological process.")

        # Check all candidate nodes
        index = np.where((gvd[1:width-1][1:height-1] == 1) \
                        & (brushfire[1:width-1][1:height-1] > 3))
        for i in range(len(index)):
            x = index[0][i]
            y = index[1][i]

            # Count neighbors at gvd inculding (x,y) node
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
                        # Boundary and gvd check
                        if x+i < 0 or x+i > width or y+j < 0 or y+j > height or gvd[x+i][y+j] == 0:
                            continue
                        if brushfire[x+i][y+j] < min:
                            notGood = True
                        if (brushfire[x+i][y+j] + 0.5) != min:
                            diff = True
                if not notGood and diff:
                    nodes.append([x,y])

        rospy.loginfo("Reevalueting nodes.")
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
                    # Boundary and gvd check
                    # Only pixels from gvd are calculated in the meanVoronoiNodeDoor
                    if x+i < 0 or x+i > width or y+j < 0 or y+j > height or gvd[x+i][y+j] == 0:
                        continue
                    sum += brushfire[x+i][y+j]
                    counter += 1
            if sum/counter - brushfire[x][y] < 3.5:
                del nodes[i]

        # Check if nodes are closer than 10 and delete them
        for i in range(len(nodes)-1, -1, -1):
            x1 = nodes[i][0]
            y1 = nodes[i][1]

            for j in range(i):
                x2 = nodes[j][0]
                y2 = nodes[j][1]
                if math.hypot(x1-x2, y1-y2) < 10:
                    del nodes[i]
                    break

        rospy.loginfo("Nodes ready!")
        return nodes
