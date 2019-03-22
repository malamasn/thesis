#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
from skimage.morphology import skeletonize, binary_closing, thin
from brushfire import Brushfire

# Class with topological functions
class Topology:
    def __init(self):
        self.brushfire = Brushfire()

    # Calculate GVD using brushfire
    def gvd(self, ogm, brushfire):
        # Set obstacles and first pixels around them as non gvd pixels
        voronoi = np.zeros(ogm.shape)
        rospy.loginfo("GVD initialized.")
        # Set free space as starting gvd and skeletonize it to get one pixel diagram
        voronoi[brushfire > 3] = 1
        voronoi = skeletonize(voronoi)
        voronoi = voronoi.astype(np.uint8)
        voronoi = binary_closing(voronoi)
        voronoi = voronoi.astype(np.uint8)
        voronoi = thin(voronoi)
        voronoi = voronoi.astype(np.uint8)

        ## DO I NEED THINNING HERE ?
        rospy.loginfo("GVD done!")
        return voronoi

    # Find useful topological nodes from gvd
    def topologicalNodes(self, ogm, brushfire, gvd):
        nodes = []
        width = ogm.shape[0]
        height = ogm.shape[1]
        rospy.loginfo("Initializing topological process.")

        # Check all candidate nodes
        index = np.where(gvd[1:width-1,1:height-1] > 0)

        for l in range(len(index[0])):
            # Needs +1 to adjust x,y at entire gvd size
            # first row & col hidden at index assignment
            x = index[0][l] + 1
            y = index[1][l] + 1

            # Count neighbors at gvd inculding (x,y) node
            count_neighbors = np.sum(gvd[x-1:x+2, y-1:y+2])

            # Add points with 1 or 3 neighbors
            if count_neighbors >= 4 or count_neighbors == 2:
                nodes.append((x,y))
            # For 2-neighbor points check with more detail
            elif count_neighbors == 3:
                notGood = False
                # diff = False
                min = brushfire[x,y]
                for i in range(-10,10):
                    for j in range(-10,10):
                        # Boundary and gvd check
                        if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1 or gvd[x+i,y+j] == 0:
                            continue
                        if brushfire[x+i,y+j] < min:
                            notGood = True
                if not notGood:
                    nodes.append((x,y))

        rospy.loginfo("Reevalueting nodes.")
        final_nodes = []
        # Keep track of door areas and visited nodes
        doors = []
        visited = []
        # Recheck nodes with 1 and 2 neighbors
        for i in range(len(nodes)-1, -1, -1):
            x = nodes[i][0]
            y = nodes[i][1]

            # Count neighbors at gvd inculding node
            count_neighbors = np.sum(gvd[x-1:x+2, y-1:y+2])

            if count_neighbors >= 4 or count_neighbors == 2:
                final_nodes.append((x,y))
            else:
                if (x,y) in visited:
                    continue
                temp = [(x,y)]
                visited.append((x,y))
                # Nodes in a tight area with almost same brush will be a door
                # Represented as a line. Keep only medium point of line
                current = [(x,y)]
                next = []
                while current != []:
                    for (xx, yy) in current:
                        for k in range(-1,2):
                            for l in range(-1,2):
                                xx_k = xx + k
                                yy_l = yy + l
                                if (xx_k, yy_l) not in nodes or (xx_k, yy_l) in visited:
                                    continue
                                c_nn = np.sum(gvd[xx_k-1:xx_k+2, yy_l-1:yy_l+2])
                                if c_nn == 3 and np.abs(brushfire[xx,yy] - brushfire[xx_k,yy_l]) <= 1:
                                    temp.append((xx_k, yy_l))
                                    visited.append((xx_k, yy_l))
                                    next.append((xx_k, yy_l))
                    current = next
                    next = []

                # Add medium of line as door node
                temp.sort()
                index = len(temp)//2
                _x, _y = temp[index]
                if not gvd[_x,_y]:
                    rospy.loginfo('ERROR! Estimated door not in gvd, (x,y):', _x,_y)
                if (_x, _y) not in final_nodes:
                    final_nodes.append((_x,_y))
                    doors.append((_x,_y))
        nodes = final_nodes[:]

        # Check if nodes are closer than 25 and delete them
        for i in range(len(nodes)-1, -1, -1):
            x1 = nodes[i][0]
            y1 = nodes[i][1]
            if (x1,y1) in doors:
                continue

            for j in range(i):
                x2 = nodes[j][0]
                y2 = nodes[j][1]
                if math.hypot(x1-x2, y1-y2) < 25:
                    del nodes[i]
                    break

        rospy.loginfo("Nodes ready!")
        return nodes

    # Discard nodes that don't represent a door
    def findDoorNodes(self, nodes, allNodes, ogm, gvd, brushfire, brushfire_instance):
        doorNodes = []
        width = gvd.shape[0]
        height = gvd.shape[1]

        # Check every node
        while nodes != []:
            node = nodes.pop()
            x = node[0]
            y = node[1]

            # print("candidate door node ", node) # DEBUG:
            ob = brushfire_instance.closestObstacleBrushfire(node, ogm)
            # Doors have always just 2 obstacle points
            if len(ob) != 2:
                continue

            # Find obstacle points' line
            obs_x = [ob[0][0], ob[1][0]]
            obs_y = [ob[0][1], ob[1][1]]
            # print('obs_x', obs_x, 'obs_y', obs_y)
            if np.abs(obs_x[0] - obs_x[1]) <= 3:  # a -> inf
                # print('a->inf') # DEBUG:
                min_y = np.min(obs_y)
                max_y = np.max(obs_y)
                mean_x = int(np.mean(obs_x))
                index = np.where(ogm[mean_x-3:mean_x+4, min_y-50:max_y+51] > 49)
                line_points = len(index[0])
                all_points =  (max_y+51-min_y+50) * (4+3)
            elif np.abs(obs_y[0] - obs_y[1]) <= 3: # a -> 0
                # print('a->0')   # DEBUG:
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                mean_y = int(np.mean(obs_y))
                index = np.where(ogm[min_x-50:max_x+51, mean_y-3:mean_y+4] > 49)
                line_points = len(index[0])
                all_points =  (max_x+51-min_x+50) * (4+3)
            else:
                # print('normal line')    # DEBUG:
                line_coeffs = np.polyfit(obs_x, obs_y, 1)
                line_points = 0
                all_points = 0
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                for xx in range(min_x-50, max_x+50):
                    yy = int(xx * line_coeffs[0] + line_coeffs[1])
                    index = np.where(ogm[xx, yy-3:yy+4] > 49)
                    line_points += len(index[0])
                    all_points += (4+3)

            perc = line_points / all_points
            # print("obstacles", ob, 'line', line_points, 'all', all_points, 'perc', perc)
            if perc >= 0.15:  # FIND RIGHT THRESHOLD
                doorNodes.append((x,y))
        return doorNodes

    # Clustering of nodes to rooms with labels
    def findRooms(self, gvd, doors, nodes_with_ids, brushfire_instance):
        roomID = -1
        visited = []
        roomType = []
        roomDoor = []
        rooms = []
        areaDoors = []

        rospy.loginfo("Starting room segmentation!")

        for door in doors:
            # Add door to visited nodes
            visited.append(door)
            # Find door's first nearest neighbors nn
            nn = brushfire_instance.gvdNeighborSplitBrushfire(door, nodes_with_ids[0], gvd)
            # Check if nodes have been visited and ignore them
            for j in range(1,-1,-1):
                for i in range(len(nn[j])-1,-1,-1):
                    if nn[j][i] in visited:
                        del nn[j][i]

            # For two sides of door
            for i in range(2):
                current_room = []
                next = []
                if len(nn[i]) == 0:
                    continue

                # Start from first neighbor
                current_room.append(nn[i][0])
                # Add rest to next
                next = nn[i][1:]

                if current_room != []:
                    # Find room nodes
                    current = current_room[:]
                    foundDoor = False
                    all_doors = []
                    all_doors.append(door)

                    while current != []:
                        for node in current:
                            if node in doors and node != door:
                                foundDoor = True
                                node_nn = brushfire_instance.gvdNeighborBrushfire(\
                                            node, nodes_with_ids[0], gvd)
                                # Check door's neighbors to find nodes of current_room
                                for n in node_nn:
                                    if n in doors or n in visited:
                                        continue
                                    n_nn =  brushfire_instance.gvdNeighborBrushfire(\
                                                n, nodes_with_ids[0], gvd)
                                    for k in n_nn:
                                        if k in current_room:
                                            visited.append(n)
                                            next.append(n)
                                            current_room.append(n)
                                            break
                            else:
                                # Find neighbors of each node
                                visited.append(node)
                                node_nn = brushfire_instance.gvdNeighborBrushfire(\
                                                node, nodes_with_ids[0], gvd)
                                # For each neighbor
                                for n in node_nn:
                                    # If is another door
                                    if n in doors and n != door:
                                        foundDoor = True
                                        if n not in all_doors:
                                            all_doors.append(i)
                                        continue
                                    # If it has not been visited
                                    if n not in visited and n not in doors:
                                        visited.append(n)
                                        next.append(n)
                                        current_room.append(n)
                        current = next
                        next = []

                    if foundDoor:
                        roomType.append(1)    # Area
                        temp = []
                        for n in all_doors:
                            index = nodes_with_ids[0].index(n)
                            temp.append(index)
                        roomID += 1
                        areaDoors.append((temp, roomID))
                    else:
                        roomType.append(0)    # Room
                        index = nodes_with_ids[0].index(door)
                        roomID += 1
                        roomDoor.append((index, roomID))

                    temp = []
                    for n in current_room:
                        index = nodes_with_ids[0].index(n)
                        temp.append(index)
                    rooms.append(temp)
        rospy.loginfo("Room segmentation finished!")
        return rooms, roomDoor, roomType, areaDoors
