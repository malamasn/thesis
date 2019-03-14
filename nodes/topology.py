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
                diff = False
                min = brushfire[x,y]
                for i in range(-10,10):
                    for j in range(-10,10):
                        # Boundary and gvd check
                        if x+i < 0 or x+i > width-1 or y+j < 0 or y+j > height-1 or gvd[x+i,y+j] == 0:
                            continue
                        if brushfire[x+i,y+j] < min:
                            notGood = True
                        if (brushfire[x+i][y+j] + 0.5) != min:
                            diff = True
                if not notGood and diff:
                    nodes.append((x,y))

        rospy.loginfo("Reevalueting nodes.")

        # Recheck nodes with 1 and 2 neighbors
        for i in range(len(nodes)-1, -1, -1):
            x = nodes[i][0]
            y = nodes[i][1]

            # Count neighbors at gvd inculding node
            count_neighbors = np.sum(gvd[x-1:x+2, y-1:y+2])

            if count_neighbors >= 4:
                continue

            # Find mean of gvd points in 20x20 neighbor area
            sum = 0
            count = 0
            for ii in range(-10,10):
                for jj in range(-10,10):
                    # Boundary and gvd check
                    # Only pixels from gvd are calculated in the meanVoronoiNodeDoor
                    if x+ii < 0 or x+ii > width-1 or y+jj < 0 or y+jj > height-1 or gvd[x+ii][y+jj] == 0:
                        continue
                    sum += brushfire[x+ii][y+jj]
                    count += 1
            if sum/count - brushfire[x][y] < 1.5:
                del nodes[i]

        # Check if nodes are closer than 10 and delete them
        for i in range(len(nodes)-1, -1, -1):
            x1 = nodes[i][0]
            y1 = nodes[i][1]


            for j in range(i):
                x2 = nodes[j][0]
                y2 = nodes[j][1]
                if math.hypot(x1-x2, y1-y2) < 25:
                    del nodes[i]
                    break
        # # Uncomment to return only 2-neighbor nodes (doorNode candidates)
        # for i in range(len(nodes)-1, -1, -1):
        #     x = nodes[i][0]
        #     y = nodes[i][1]
        #     count_neighbors = np.sum(gvd[x-1:x+2, y-1:y+2])
        #     if count_neighbors != 3:
        #         del nodes[i]


        rospy.loginfo("Nodes ready!")
        return nodes

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
            if brushfire[node] > 25:    # MAYBE 30 ?
                continue

            # Check for obstacles in the 4 main directions
            north = np.any(brushfire[x,y+1:y+21] == 1)
            west = np.any(brushfire[x-20:x,y] == 1)
            south = np.any(brushfire[x,y-20:y] == 1)
            east = np.any(brushfire[x+1:x+21,y] == 1)

            # If not 2 sequential obstacles found, it is a door
            if not (east and south or south and west or west and north or north and east):
                doorNodes.append((x,y))

            # Check for obstacles in the 4 diagonal directions
            north = False
            west = False
            south = False
            east = False

            for i in range(1,21):

                if brushfire[x+i,y+i] == 1:
                    east = True
                if brushfire[x-i,y+i] == 1:
                    north = True
                if brushfire[x-i,y-i] == 1:
                    west = True
                if brushfire[x+i,y-i] == 1:
                    south = True

            # If not 2 sequential obstacles found, it is a door
            if not (east and south or south and west or west and north or north and east):
                doorNodes.append((x,y))

        return doorNodes

    # Clustering of nodes to rooms with labels
    def findRooms(self, gvd, doors, nodes_with_ids, brushfire_instance):
        visited = []
        roomType = []
        roomDoor = []
        rooms = []
        areaDoors = []

        rospy.loginfo("Starting room segmentation!")

        for door in doors:
            # Add door to visited nodes
            print('door', door) # DEBUG:
            visited.append(door)
            # Find door's first nearest neighbors nn
            nn = brushfire_instance.gvdNeighborSplitBrushfire(door, nodes_with_ids[0], gvd)
            print('nn start', nn) # DEBUG:

            # Check if nodes have been visited and ignore them
            for j in range(1,-1,-1):
                for i in range(len(nn[j])-1,-1,-1):
                    if nn[j][i] in visited:
                        del nn[j][i]
                    else:
                        visited.append(nn[j][i])
            print('nn after visited test', nn) # DEBUG:
            print('visited', visited)
            for i in range(2):
                current_room = []
                # Find closest neighbor to door and add to next
                print('len',len(nn[i]), nn[i])
                # if len(nn[i]) == 0:
                #     continue
                if len(nn[i]) == 1:
                    current_room.append(nn[i][0])
                    next_1 = nn[i][1:]
                elif len(nn[i]) > 1:
                    # find closest to door
                    nn_array = np.array(nn[i])
                    door_array = np.array(door)
                    closest = np.linalg.norm(nn_array-door_array, axis=1).argmin()
                    # closest = 0
                    first = tuple(nn_array[closest])
                    current_room.append(first)
                    nn[i].remove(first)
                    # Add rest door neighbors to next_1
                    next_1 = nn[i]

                print('current_room', current_room)   # DEBUG:
                if current_room != []:
                    # Find room nodes
                    current = current_room[:]
                    next = next_1[:]
                    foundDoor = False
                    all_doors = []
                    all_doors.append(door)
                    while current != []:
                        for node in current:
                            if node in doors and node != door:
                                foundDoor = True
                            # Find neighbors of each node
                            node_nn = brushfire_instance.gvdNeighborBrushfire(node, nodes_with_ids[0], gvd)
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
                        areaDoors.append(temp)
                    else:
                        roomType.append(0)    # Room
                        index = nodes_with_ids[0].index(door)
                        roomDoor.append(index)
                    rooms.append(current_room)
                print('rooms', rooms)

        rospy.loginfo("Room segmentation finished!")
        return rooms, roomDoor, roomType, areaDoors
