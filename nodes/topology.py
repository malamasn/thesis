#!/usr/bin/env python
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

        for i in range(len(index[0])):
            # Needs +1 to adjust x,y at entire gvd size
            # first row & col hidden at index assignment
            x = index[0][i] + 1
            y = index[1][i] + 1

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
                        # if (brushfire[x+i][y+j] + 0.5) != min:
                        #     diff = True
                if not notGood: # and diff:
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
                for j in range(len(nodes)):
                    xx = nodes[j][0]
                    yy = nodes[j][1]
                    if (xx,yy) in visited:
                        continue
                    c_nn = np.sum(gvd[xx-1:xx+2, yy-1:yy+2])
                    if c_nn == 3 and np.abs(brushfire[x,y] - brushfire[xx,yy]) <= 1\
                        and (np.abs(x-xx) < 2 or np.abs(y-yy) < 2):
                        temp.append((xx,yy))
                        visited.append((xx,yy))
                # Doors will have plenty of points
                if len(temp) <= 10:
                    continue
                # Add mean of line as door node
                mean = np.mean(temp, axis=0)
                _x = int(mean[0])
                _y = int(mean[1])
                if not gvd[_x,_y]:
                    temp.sort()
                    index = len(temp)/2
                    _x, _y = temp[index]
                if not gvd[_x,_y]:
                    print('Estimated door not in gvd, (x,y):', _x,_y)
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
    def findDoorNodes(self, nodes, allNodes, gvd, brushfire):
        doorNodes = []
        width = gvd.shape[0]
        height = gvd.shape[1]

        # Check every node
        while nodes != []:
            node = nodes.pop()
            x = node[0]
            y = node[1]
            # Nodes far from obstacles usually are not obstacles
            if brushfire[node] <= 25:   # MAYBE 30 ?
                doorNodes.append(node)
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
                                for i in node_nn:
                                    if i in doors or i in visited:
                                        continue
                                    i_nn =  brushfire_instance.gvdNeighborBrushfire(\
                                                i, nodes_with_ids[0], gvd)
                                    for k in i_nn:
                                        if k in current_room:
                                            visited.append(i)
                                            next.append(i)
                                            current_room.append(i)
                                            break
                            else:
                                # Find neighbors of each node
                                visited.append(node)
                                node_nn = brushfire_instance.gvdNeighborBrushfire(\
                                                node, nodes_with_ids[0], gvd)
                                # For each neighbor
                                for i in node_nn:
                                    # If is another door
                                    if i in doors and i != door:
                                        foundDoor = True
                                        if i not in all_doors:
                                            all_doors.append(i)
                                        continue
                                    # If it has not been visited
                                    if i not in visited and i not in doors:
                                        visited.append(i)
                                        next.append(i)
                                        current_room.append(i)
                        current = next
                        next = []

                    if foundDoor:
                        roomType.append(1)    # Area
                        temp = []
                        for i in all_doors:
                            index = nodes_with_ids[0].index(i)
                            temp.append(index)
                        roomID += 1
                        areaDoors.append((temp, roomID))
                    else:
                        roomType.append(0)    # Room
                        index = nodes_with_ids[0].index(door)
                        roomID += 1
                        roomDoor.append((index, roomID))

                    temp = []
                    for i in current_room:
                        index = nodes_with_ids[0].index(i)
                        temp.append(index)
                    rooms.append(temp)
        rospy.loginfo("Room segmentation finished!")
        return rooms, roomDoor, roomType, areaDoors
