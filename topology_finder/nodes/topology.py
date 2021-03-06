#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import pandas as pd
import math, os
from skimage.morphology import skeletonize, binary_closing, thin
from brushfire import Brushfire

from sklearn.externals import joblib

# Class with topological functions
class Topology:
    def __init(self):
        self.brushfire = Brushfire()

    # Calculate GVD using brushfire
    def gvd(self, brushfire):
        # Set obstacles and first pixels around them as non gvd pixels
        voronoi = np.zeros(brushfire.shape)
        rospy.loginfo("GVD initialized.")
        # Set free space as starting gvd and skeletonize it to get one pixel diagram
        voronoi[brushfire >= 5] = 1
        voronoi = skeletonize(voronoi)
        voronoi = voronoi.astype(np.uint8)
        # voronoi = binary_closing(voronoi)
        # voronoi = voronoi.astype(np.uint8)
        # voronoi = thin(voronoi)
        # voronoi = voronoi.astype(np.uint8)

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

        # # Check if nodes are closer than 25 and delete them
        # for i in range(len(nodes)-1, -1, -1):
        #     x1 = nodes[i][0]
        #     y1 = nodes[i][1]
        #     if (x1,y1) in doors:
        #         continue
        #
        #     for j in range(i):
        #         x2 = nodes[j][0]
        #         y2 = nodes[j][1]
        #         if math.hypot(x1-x2, y1-y2) < 25:
        #             del nodes[i]
        #             break

        rospy.loginfo("Nodes ready!")
        return nodes

    # Discard nodes that don't represent a door
    def findDoorNodes(self, nodes, ogm, gvd, brushfire_instance):
        doorNodes = []
        width = gvd.shape[0]
        height = gvd.shape[1]
        perc = []
        candidateDoors = []
        rospy.loginfo("Start finding door nodes!")
        # Check every node
        for node in nodes:
            # node = nodes.pop()
            x = node[0]
            y = node[1]

            # print("candidate door node ", node) # DEBUG:
            ob = brushfire_instance.closestObstacleBrushfireCffi(node, ogm)
            # Doors have always just 2 obstacle points
            if len(ob) != 2:
                continue
            # Door nodes should always be inside the obstacles' line area
            if x < np.min([ob[0][0], ob[1][0]])-5 or  x > np.max([ob[0][0], ob[1][0]])+5:
                continue
            if y < np.min([ob[0][1], ob[1][1]])-5 or  y > np.max([ob[0][1], ob[1][1]])+5:
                continue
            candidateDoors.append(node)

            # Find obstacle points' line
            obs_x = [ob[0][0], ob[1][0]]
            obs_y = [ob[0][1], ob[1][1]]
            # print('obs_x', obs_x, 'obs_y', obs_y)
            if np.abs(obs_x[0] - obs_x[1]) <= 3:  # a -> inf
                # print('a->inf') # DEBUG:
                min_y = np.min(obs_y)
                max_y = np.max(obs_y)
                mean_x = int(np.mean(obs_x))
                index = np.where(ogm[mean_x-2:mean_x+3, min_y-50:max_y+51] > 49)
                line_points = len(index[0])
                all_points =  (max_y+51-min_y+50) * (2+3)
            elif np.abs(obs_y[0] - obs_y[1]) <= 3: # a -> 0
                # print('a->0')   # DEBUG:
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                mean_y = int(np.mean(obs_y))
                index = np.where(ogm[min_x-50:max_x+51, mean_y-2:mean_y+3] > 49)
                line_points = len(index[0])
                all_points =  (max_x+51-min_x+50) * (2+3)
            else:
                # print('normal line')    # DEBUG:
                line_coeffs = np.polyfit(obs_x, obs_y, 1)
                line_points = 0
                all_points = 0
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                for xx in range(min_x-50, max_x+50):
                    if xx < 0 or xx >= width:
                        continue
                    yy = int(xx * line_coeffs[0] + line_coeffs[1])
                    if yy < 0 or yy >= height:
                        continue
                    index = np.where(ogm[xx, yy-2:yy+3] > 49)
                    line_points += len(index[0])
                    all_points += (2+3)

            perc.append(line_points / all_points)
            # print("obstacles", ob, 'line', line_points, 'all', all_points, 'perc', line_points / all_points)
        # that case if all nodes in perc are doors, they will be all set as doorNodes
        perc = np.append(perc, 0.0)
        perc_copy = np.array(perc)
        perc_copy.sort()
        if len(perc_copy) > 1:
            diff = np.diff(perc_copy)
            # print('perc', perc_copy)
            max_index = diff.argmax()
            threshold = perc_copy[max_index]
            max_values = [np.max(diff)]
            while threshold > 0.20:
                max = 0
                current_index = -1
                for i in range(len(diff)):
                    if diff[i] > max and diff[i] not in max_values:
                        max = diff[i]
                        current_index = i
                max_values.append(max)
                threshold = perc_copy[current_index]
            # if threshold < 0.10:
            #     max_index = diff.argmax()
            #     threshold = perc_copy[max_index]
            #     max_values = [np.max(diff)]
            #     while threshold > 0.30:
            #         max = 0
            #         current_index = -1
            #         for i in range(len(diff)):
            #             if diff[i] > max and diff[i] not in max_values:
            #                 max = diff[i]
            #                 current_index = i
            #         max_values.append(max)
            #         threshold = perc_copy[current_index]
        elif len(perc_copy) == 1:
            threshold = perc_copy[0] - 0.001
        else:
            rospy.loginfo("No doors found!")
            return doorNodes
        # print('threshold', threshold)
        rospy.loginfo("Comparing nodes with threshold.")
        for i in range(len(perc)):
            if perc[i] > threshold:
                doorNodes.append(candidateDoors[i])
        rospy.loginfo("Door nodes found!")
        return doorNodes

    # Fill door holes with wall
    def doorClosure(self, doors, ogm, brushfire_instance):
        filled_ogm = np.copy(ogm)
        width = ogm.shape[0]
        height = ogm.shape[1]

        for door in doors:
            ob = brushfire_instance.closestObstacleBrushfireCffi(tuple(door), ogm)
            # Find obstacle points' line
            obs_x = [ob[0][0], ob[1][0]]
            obs_y = [ob[0][1], ob[1][1]]
            # For all line cases fill
            if np.abs(obs_x[0] - obs_x[1]) <= 3:  # a -> inf
                min_y = np.min(obs_y)
                max_y = np.max(obs_y)
                mean_x = int(np.mean(obs_x))
                filled_ogm[mean_x-1:mean_x+2, min_y:max_y] = 100

            elif np.abs(obs_y[0] - obs_y[1]) <= 3: # a -> 0
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                mean_y = int(np.mean(obs_y))
                filled_ogm[min_x:max_x, mean_y-1:mean_y+2] = 100

            else:
                line_coeffs = np.polyfit(obs_x, obs_y, 1)
                line_points = 0
                all_points = 0
                min_x = np.min(obs_x)
                max_x = np.max(obs_x)
                for xx in range(min_x, max_x):
                    if xx < 0 or xx >= width:
                        continue
                    yy = int(xx * line_coeffs[0] + line_coeffs[1])
                    if yy < 0 or yy >= height:
                        continue
                    filled_ogm[xx, yy-1:yy+2] = 100
        return filled_ogm

    # Clustering of nodes to rooms with labels
    def findRooms(self, gvd, doors, nodes, brushfire, ogm, resolution, brushfire_instance):
        visited = []

        rooms = []
        roomType = []
        roomDoors = []

        rospy.loginfo("Start room segmentation!")

        for door in doors:
            # Add door to visited nodes
            visited.append(door)
            # Find door's first nearest neighbors nn
            nn = brushfire_instance.gvdNeighborSplitBrushfireCffi(tuple(door), nodes, gvd)
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
                                node_nn = brushfire_instance.gvdNeighborBrushfireCffi(\
                                            node, nodes, gvd)
                                # Check door's neighbors to find nodes of current_room
                                for n in node_nn:
                                    if n in doors or n in visited:
                                        continue
                                    n_nn =  brushfire_instance.gvdNeighborBrushfireCffi(\
                                                n, nodes, gvd)
                                    for k in n_nn:
                                        if k in current_room:
                                            visited.append(n)
                                            next.append(n)
                                            current_room.append(n)
                                            break
                            else:
                                # Find neighbors of each node
                                visited.append(node)
                                node_nn = brushfire_instance.gvdNeighborBrushfireCffi(\
                                                node, nodes, gvd)
                                # For each neighbor
                                for n in node_nn:
                                    # If is another door
                                    if n in doors and n != door:
                                        foundDoor = True
                                        if n not in all_doors:
                                            all_doors.append(n)
                                        continue
                                    # If it has not been visited
                                    if n not in visited and n not in doors:
                                        visited.append(n)
                                        next.append(n)
                                        current_room.append(n)
                        current = next
                        next = []

                    # FOLLOWING NOT YET CORRECT AS A SEGMENTATION
                    if foundDoor:
                        roomType.append(2)    # Area
                    else:
                        roomType.append(0)    # Room

                    roomDoors.append(all_doors)
                    rooms.append(current_room)

        rospy.loginfo("Room segmentation finished!")

        # rospy.loginfo("Start collecting room data!")
        # size = len(rooms)
        # attribute_size = 7
        # data = np.zeros((size,attribute_size))
        # for i in range(size):
        #     # Number of nodes
        #     data[i][0] = len(rooms[i])
        #     indexes = zip(*rooms[i])
        #     # Brushfire mean of nodes
        #     data[i][1] = resolution * p.sum(brushfire[indexes])/len(rooms[i])
        #     # Standard deviation of brushfire
        #     data[i][2] = resolution * p.std(brushfire[indexes])
        #     # Mean of number of nodes' neighbors
        #     total_neighbors = 0
        #     for x,y in rooms[i]:
        #         total_neighbors += np.sum(gvd[x-1:x+2, y-1:y+2]) - 1
        #     total_neighbors /= len(rooms[i])
        #     data[i][3] = total_neighbors
        #     # Mean distance of nodes
        #     total_distance = 0
        #     for x in range(len(rooms[i])):
        #         for y in range(x+1, len(rooms[i])):
        #             dist = np.linalg.norm(np.array(rooms[i][x])-np.array(rooms[i][y]))
        #             total_distance += dist
        #     data[i][4] = resolution * total_distance/np.sum(range(len(rooms[i])))
        #     # Mean minimum distance
        #     min = np.zeros((len(rooms[i])))
        #     min[:] = np.inf
        #     for x in range(len(rooms[i])):
        #         nn = brushfire_instance.gvdNeighborBrushfireCffi(\
        #                         rooms[i][x], nodes, gvd)
        #         for node_nn in nn:
        #             dist = np.linalg.norm(np.array(node_nn)-np.array(rooms[i][x]))
        #             if dist < min[x]:
        #                 min[x] = dist
        #     data[i][5] = resolution * np.sum(min)/len(rooms[i])
        #     # If there are no room neighbors set mean distance as mean min distance
        #     if len(rooms[i]) == 1 or total_distance == 0:
        #         data[i][4] = data[i][5]
        #     # Class attribute, depends on map
        #     data[i][attribute_size-1] = 0
        # filename = 'new_data.csv'
        # file_exists = os.path.isfile(filename)
        # if not file_exists:
        #     df = pd.DataFrame(data, columns=['Number of Nodes', 'Brushfire mean',
        #                     'Standard deviation of Brushfire', 'NNs mean', 'Mean distance',
        #                     'Mean minimun distance', 'Class'])
        # else:
        #     df_old = pd.read_csv(filename)
        #     df = df_old.copy()
        #     df_new = pd.DataFrame(data)
        #     df = df.append(df_new)
        # # csv name is a generic name
        # df.to_csv(filename, index=False)

        # rospy.loginfo("Data saved to csv!")

        # rospy.loginfo("Predicting room types.")
        # filename = '/home/mal/catkin_ws/src/room_classification/models/room_classifier.sav'
        # model = joblib.load(filename)
        # for i in range(len(rooms)):
        #     # Areas with one door are set as rooms
        #     if roomType[i] == 2:
        #         x = np.zeros((1,6))
        #         # Number of nodes
        #         x[0][0] = len(rooms[i])
        #         if not x[0][0]:
        #             roomType[i] = 0
        #             continue
        #         indexes = zip(*rooms[i])
        #         # Brushfire mean of nodes
        #         x[0][1] = resolution * np.sum(brushfire[indexes])/len(rooms[i])
        #         # Standard deviation of brushfire
        #         x[0][2] = resolution * np.std(brushfire[indexes])
        #         # Mean of number of nodes' neighbors
        #         total_neighbors = 0
        #         for xx,yy in rooms[i]:
        #             total_neighbors += np.sum(gvd[xx-1:xx+2, yy-1:yy+2]) - 1
        #         x[0][3] = total_neighbors/len(rooms[i])
        #         # Mean distance of nodes
        #         total_distance = 0
        #         for xx in range(len(rooms[i])):
        #             for yy in range(xx+1, len(rooms[i])):
        #                 dist = np.linalg.norm(np.array(rooms[i][xx])-np.array(rooms[i][yy]))
        #                 total_distance += dist
        #         x[0][4] = resolution * total_distance/np.sum(range(len(rooms[i])))
        #         # Mean minimum distance
        #         min = np.zeros((len(rooms[i])))
        #         min[:] = np.inf
        #         for xx in range(len(rooms[i])):
        #             nn = brushfire_instance.gvdNeighborBrushfireCffi(\
        #                             rooms[i][xx], nodes, gvd)
        #             for node_nn in nn:
        #                 dist = np.linalg.norm(np.array(node_nn)-np.array(rooms[i][xx]))
        #                 if dist < min[xx]:
        #                     min[xx] = dist
        #         x[0][5] = resolution * np.sum(min)/len(rooms[i])
        #         if len(rooms[i]) == 1 or total_distance == 0:
        #             x[0][4] = x[0][5]
        #         # Predict room type
        #         roomType[i] = int(model.predict(x))

        rospy.loginfo("Room finding process done!")
        return rooms, roomDoors, roomType
