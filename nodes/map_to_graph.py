#!/usr/bin/env python
from __future__ import division
import rospy, json
import numpy as np
import time, math
from dijkstar import Graph, find_path
from operator import itemgetter
from scipy.spatial.distance import cdist

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from utilities import Cffi
from topology import Topology
from routing import Routing
from coverage import Coverage
from PIL import Image


class Map_To_Graph:

    def __init__(self):
        self.brushfire_cffi = Cffi()
        self.topology = Topology()
        self.routing = Routing()
        self.coverage = Coverage()

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.resolution = 0

        # Load map's translation
        translation = rospy.get_param('origin')
        self.origin['x'] = translation[0]
        self.origin['y'] = translation[1]
        self.resolution = rospy.get_param('resolution')

        # Initilize sensor's specs
        self.sensor_names = []
        self.sensor_direction = []
        self.sensor_fov = []
        self.sensor_range = []
        self.sensor_shape = []
        self.sensor_reliability = []
        self.min_distance = 0

        # Read sensor's specs
        self.min_distance = rospy.get_param('min_distance')
        self.sensor_names = rospy.get_param('sensor_names')
        self.sensor_number = len(self.sensor_names)
        for name in self.sensor_names:
            self.sensor_direction.append(rospy.get_param(name + '/sensor_direction'))
            self.sensor_fov.append(rospy.get_param(name + '/fov'))
            self.sensor_range.append(rospy.get_param(name + '/range'))
            self.sensor_shape.append(rospy.get_param(name + '/shape'))
            self.sensor_reliability.append(rospy.get_param(name + '/reliability'))

        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        # Flag to wait ogm subscriber to finish
        self.ogm_compute = False

        self.gvd = 0
        self.brush = 0

        self.nodes = []
        self.door_nodes = []
        self.rooms = []
        self.room_doors = []
        self.room_type = []
        self.room_sequence = []
        self.entering_doors = {}
        self.exiting_doors = {}
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        # Load nodes from json file
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'r') as read_file:
            self.data = json.load(read_file)

        self.nodes = self.data['nodes']
        self.door_nodes = self.data['doors']
        self.rooms = self.data['rooms']
        self.room_doors = self.data['roomDoors']
        self.room_type = self.data['roomType']

        if 'room_sequence' in self.data:
            self.room_sequence = self.data['room_sequence']
            self.entering_doors = { int(k):v for k,v in self.data['entering_doors'].items() }
            self.exiting_doors = { int(k):v for k,v in self.data['exiting_doors'].items() }

        if 'wall_follow_nodes' in self.data:
            self.wall_follow_nodes = self.data['wall_follow_nodes']

        if 'wall_follow_sequence' in self.data:
            self.wall_follow_sequence = self.data['wall_follow_sequence']

        # if 'boustrophedon_sequence' in self.data:
        #     self.boustrophedon_sequence = self.data['boustrophedon_sequence']

        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)

    def server_start(self):
        rospy.init_node('map_to_graph')
        rospy.loginfo('map_to_graph node initialized.')


        # Read ogm
        self.ogm_compute = True
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        while self.ogm_compute:
            pass

        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)
        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.ogm, self.brush)

        time.sleep(1)
        # self.print_markers(self.nodes, [1., 0., 0.], self.node_publisher)
        # self.print_markers(self.door_nodes, [0., 0., 1.], self.door_node_pub)

        # Find sequence of rooms if not already exists
        if self.room_sequence == []:
            self.find_room_sequence(True)

        # time.sleep(1)
        # # i = 0
        # # route = self.data['room_sequence']
        # for _ in range(1):
        #     for room_id in self.room_sequence:
        #         self.print_markers(self.rooms[room_id], [1.0, 0., 1.0], self.room_node_pub)
        #         print('room type: ', self.room_type[room_id])
        #         # i += 1
        #         time.sleep(3)
#
        # Find nodes for wall following coverage
        # if self.wall_follow_nodes == [] or self.wall_follow_sequence == []:
        # self.find_all_wall_nodes(True)
        self.find_best_path_wall_nodes(True)
        # self.find_half_wall_nodes(True)
        # self.find_half_no_double_wall_nodes(True)

        # self.visualise_node_sequence(self.wall_follow_sequence)
        self.wall_follow_sequence = self.a_priori_coverage(self.wall_follow_sequence)

        # Save wall nodes to json
        self.data['wall_follow_nodes'] = self.wall_follow_nodes
        self.data['wall_follow_sequence'] = self.wall_follow_sequence

        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
            data_to_json = json.dump(self.data, outfile)

        rospy.loginfo("Visualize node sequence")
        self.visualise_node_sequence(self.wall_follow_sequence)

        return

    # Method to visualize sequence of nodes splited into rooms with different color
    def visualise_node_sequence(self, rooms):
        id = 0
        for nodes in rooms:
            rgb = [0,0,0]
            i = 0
            for node in nodes:
                rgb[0] = i/len(nodes)
                rgb[1] = min(2*i,len(nodes))/len(nodes)
                rgb[2] = min(3*i,len(nodes))/len(nodes)
                self.print_markers([node['position']],rgb, self.room_node_pub, id)
                rospy.sleep(0.01)
                i += 1
                id += 1
            print("Printed entire room")
        return

    # Method to visualize room nodes splited into segments with different color in each one
    def visualise_node_segments(self, room, id = 0):
        color_num = 0
        color = [1.,0.,0.]
        # threshold = 256/len(splited_node_route)
        for segment in room:
            self.print_markers(segment, np.array(color_num)*color, self.room_node_pub, id)
            id += 1
            color_num += 30/256
            if color_num > 1:
                color = (color[1:]+color[:1])
                color_num -= 1
            rospy.sleep(0.1)
        return  id

    # Do uniform sampling across whole map and gather points near obstacles
    def uniform_sampling(self):
        # Uniform sampling on map
        nodes = []
        # Sampling step is half the sensor's range
        min_range = min(self.sensor_range)
        min_range_res = min_range / (self.resolution * 2)
        # max_range = max(self.sensor_range)
        # max_range_res = max_range / (self.resolution * 2)

        step = int(min_range /(self.resolution * 2))
        # step = int(min_range /(self.resolution * 1))

        if step < self.min_distance:
            loginfo("Error. Uniform sampling step size too small!")
            return

        for x in range(0, self.ogm_width, step):
            for y in range(0, self.ogm_height, step):
                # Node should be close to obstacle, but not too close to avoid collision
                if self.brush[x][y] > min_range_res and self.brush[x][y] <= 2*min_range_res:
                    nodes.append((x,y))
        return nodes, step

    # Find sequence of rooms
    def find_room_sequence(self, save_result):
        rospy.loginfo("Finding room sequence...")
        self.room_sequence = []
        # Create graph
        graph = Graph()
        for room in self.room_doors:
            if len(room) > 1:
                for i in range(len(room)):
                    for j in range(i+1, len(room)):
                        node_id_1 = self.door_nodes.index(room[i])
                        node_id_2 = self.door_nodes.index(room[j])
                        dist = np.linalg.norm(np.array(room[i])- np.array(room[j]))
                        graph.add_edge(node_id_1, node_id_2, dist)
                        graph.add_edge(node_id_2, node_id_1, dist)

        # Calculate graph's distances between all nodes
        doors_length = len(self.door_nodes)
        if doors_length >= 2:
            distances = np.zeros((doors_length, doors_length), dtype=np.float64)
            for i in range(doors_length):
                for j in range(i+1, doors_length):
                    _, _, _, dist = find_path(graph, i, j)
                    distances[i][j] = dist
                    distances[j][i] = distances[i][j]

            # Call hill climb algorithm
            max_iterations = 500
            # door_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
            # door_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations)
            door_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
            # print(door_route, cost, iter)
        else:
            door_route = [0]

        # Correspond door route to room route
        for door in door_route:
            for i in range(len(self.room_doors)):
                if self.door_nodes[door] in self.room_doors[i] and i not in self.room_sequence:
                    self.room_sequence.append(i)

        enter = {}
        leave = {}
        if doors_length >= 2:
            # Compute enter & exit doors of each room according to found room_sequence
            room_i = 1  # Start from 2nd room
            room_idx = self.room_sequence[room_i]
            entered = False
            # Simulate door/room sequence on the graph
            for i in range(len(door_route)):
                # Find path from current to next door
                door_idx = door_route[i]
                next_door_idx = door_route[(i+1)%len(door_route)]
                # print('Room', room_idx)
                path = find_path(graph, door_idx, next_door_idx)
                # print('Path',path.nodes)
                if entered and len(path.nodes) > 1: # if already inside room, first node of path is already used
                    ii = 1
                else:
                    ii = 0
                # Transverse through path and check for corresponding rooms
                while ii < len(path.nodes):
                    node_idx = path.nodes[ii]
                    door = self.door_nodes[node_idx]
                    if door in self.room_doors[room_idx]:
                        # print('Door',node_idx, door,'in room', room_idx)
                        if not entered:
                            enter[room_idx] = door
                            entered = not entered
                            if len(self.room_doors[room_idx]) == 1:
                                ii -= 1     # To revisit this door
                        else:
                            leave[room_idx] = door
                            entered = not entered
                            room_i += 1
                            room_idx = self.room_sequence[room_i%len(self.room_sequence)]
                            # print('Room changed', room_idx)
                            ii -= 1     # To revisit this door
                    ii += 1
        else:
            for i in range(len(self.room_sequence)):
                enter[i] = self.door_nodes[0]
                leave[i] = self.door_nodes[0]

        self.entering_doors = enter
        self.exiting_doors = leave


        # # Print results per room (blue - entering door, read - exiting door, green - room nodes)
        # for room_idx in self.room_sequence:
        #     print('Room',room_idx, 'all doors',self.room_doors[room_idx])
        #     self.print_markers([enter[room_idx]], [0,0,1], self.node_publisher)
        #     self.print_markers([leave[room_idx]], [1,0,0], self.door_node_pub)
        #     self.print_markers(self.rooms[room_idx], [0,1,0], self.room_node_pub)
        #     rospy.sleep(4)

        if save_result:
            # Save room sequence
            self.data['room_sequence'] = self.room_sequence
            self.data['entering_doors'] = self.entering_doors
            self.data['exiting_doors'] = self.exiting_doors
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

        return

    # Find the yaw that maximizes the covered walls of a given node
    def find_best_yaw(self, node, next_node, loop_threshold, obstacle_weight, rotation_weight):
        yaw_between_nodes = math.degrees(math.atan2(next_node[1]-node[1], next_node[0]-node[0]))
        yaw = []

        steps = int(max(self.sensor_range) / self.resolution)
        # Find reachable obstacles
        obstacles = self.brushfire_cffi.inRangeObstacleBrushfireCffi(node, self.ogm, steps, True)

        if len(obstacles) == 0:
            return yaw

        # In case of many obstacles more poses are needed for full coverage
        found = 0
        while found < loop_threshold * len(obstacles) or len(obstacles):

            best_evaluation = 0
            best_yaw = 0
            best_found = 0
            best_covered_obstacles = []
            for angle in range(-180, 180, 10):
                pose = [node[0], node[1], angle]
                all_covered_obstacles = self.coverage.checkNearbyObstacleCover(pose)
                covered_obstacles = []
                for ob in all_covered_obstacles:
                    if ob in obstacles:
                        covered_obstacles.append(ob)
                if not len(covered_obstacles):
                    continue

                # Absolute yaw of current and next node in [0,180]
                next_rotation = np.abs(angle - yaw_between_nodes)
                while next_rotation >= 360:
                    next_rotation -= 360
                if next_rotation > 180:
                    next_rotation = 360 - next_rotation

                # Evaluate candidate angle
                # Use Motor Schema with covered area and rotation to next node as parameters
                evaluation = obstacle_weight * len(covered_obstacles)/len(obstacles) + rotation_weight * (180 - next_rotation)/180
                if evaluation > best_evaluation:
                    best_evaluation = evaluation
                    best_yaw = angle
                    best_found = len(covered_obstacles)
                    best_covered_obstacles = covered_obstacles

            found += best_found

            # Check if obstacle is coverable
            if best_evaluation:
                yaw.append(best_yaw)
            else:
                break

            # Speed up process bypassing deletions if all obstacles are covered
            if found >= loop_threshold * len(obstacles):
                break
            else:
                # Discard checked obstacles
                for ob in best_covered_obstacles:
                    if ob in obstacles:
                        obstacles.remove(ob)

        yaw = list(set(yaw))
        return yaw

    # Find nodes for wall following coverage
    def find_all_wall_nodes(self, save_result):
        rospy.loginfo("Finding wall follow nodes...")
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        # Uniform sampling on map
        nodes, step = self.uniform_sampling()

        self.print_markers(nodes, [1., 0., 0.], self.node_publisher)
        rospy.sleep(2)

        # Find rooms of nodes
        # Fill door holes with "wall"
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)

        for i in range(len(self.rooms)):
            # Brushfire inside each room and gather brushed wall_follow_nodes
            found_nodes = []
            brush = self.brushfire_cffi.pointBrushfireCffi(self.rooms[i], filled_ogm)
            for n in nodes:
                if brush[n] > 0:
                    found_nodes.append(n)
            found_nodes.sort()  # Optimize path finding
            print("Found {} nodes in room {}.".format(len(found_nodes), i))  # # DEBUG
            # found_nodes.reverse()
            nodes_length = len(found_nodes)
            distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
            print('Distances of nodes done.')    # DEBUG:

            # Call hill climb algorithm
            max_iterations = 150 * nodes_length
            # node_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
            node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, step)
            print('Route of wall follow nodes found.')

            found_nodes_with_yaw = []
            k = 1   # DEBUG:

            for n in range(len(found_nodes)):

                print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                # Find closest obstacle to get best yaw
                x, y = found_nodes[node_route[n]]
                x2, y2 = found_nodes[node_route[(n+1)%len(node_route)]]
                yaw = self.find_best_yaw((x,y), (x2,y2))
                if yaw == []:
                    continue
                for point in yaw:
                    temp_dict = {'position': (x,y), 'yaw': point}
                    found_nodes_with_yaw.append(temp_dict)
                k += 1

            # print(found_nodes_with_yaw)
            self.wall_follow_nodes.append(found_nodes)
            self.wall_follow_sequence.append(found_nodes_with_yaw)

            # for i in range(len(found_nodes)):
            #     node = found_nodes[node_route[i]]
            #     self.print_markers([node], [0., 0., 1.], self.room_node_pub)
            #     rospy.sleep(0.5)
            # rospy.sleep(2)


        if save_result:
            # Save wall nodes to json
            self.data['wall_follow_nodes'] = self.wall_follow_nodes
            self.data['wall_follow_sequence'] = self.wall_follow_sequence
            # self.data['boustrophedon_sequence'] = self.boustrophedon_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

        return

    # Find nodes for wall following coverage with optimal path
    def find_best_path_wall_nodes(self, save_result):
        rospy.loginfo("Finding wall follow nodes...")
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        loop_threshold, obstacle_weight, rotation_weight = 0.6, 2, 1

        # Uniform sampling on map
        nodes, step = self.uniform_sampling()

        # self.print_markers(nodes, [1., 0., 0.], self.node_publisher)
        rospy.sleep(1)

        # Find rooms of nodes
        # Fill door holes with "wall"
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)
        id = 0
        for i in range(0, len(self.rooms)):
            # Brushfire inside each room and gather brushed wall_follow_nodes
            found_nodes = []
            brush = self.brushfire_cffi.pointBrushfireCffi(self.rooms[i], filled_ogm)
            for n in nodes:
                if brush[n] > 0:
                    found_nodes.append(n)
            found_nodes.sort()  # Optimize path finding
            print("Found {} nodes in room {}.".format(len(found_nodes), i))  # # DEBUG

            if len(found_nodes) == 0:
                print("ERROR!")
                continue

            nodes_length = len(found_nodes)
            distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
            print('Distances of nodes done.')    # DEBUG:
            cost = self.routing.route_length(distances, range(nodes_length))
            print('Sorted route length', cost)

            # Call hill climb algorithm
            max_iterations = 150 * nodes_length
            node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, step)
            print('Route of wall follow nodes found.')
            print('First step HC route length', cost)
            #
            # final_route = []
            # for n in node_route:
            #     final_route.append(found_nodes[n])

            # Reorder according to step HC results
            first_route = []
            first_route.append(self.entering_doors[i])
            for n in node_route:
                first_route.append(found_nodes[n])
            first_route.append(self.exiting_doors[i])

            # Do another hillclimb to optimize path
            max_iterations = 1000 * len(first_route)
            distances = cdist(np.array(first_route), np.array(first_route), 'euclidean')
            # node_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
            fixed_edges = True
            node_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations, fixed_edges)
            print('Second HC route length', cost, 'made iters', iter, 'from', max_iterations)

            final_route = []
            for n in node_route:
                final_route.append(first_route[n])

            # # Split route into straight segments
            # splited_node_route = []
            # i = 0
            # threshold = int(len(node_route) / 10)
            # while i < len(node_route):
            #     segment = []
            #     segment.append(found_nodes[node_route[i]])
            #     i += 1
            #     flag = False
            #     while i < len(node_route) and not flag:
            #         x, y = found_nodes[node_route[i]]
            #         x_prev, y_prev = found_nodes[node_route[i-1]]
            #         # If node is far from previous, a new segment is needed
            #         if np.linalg.norm(np.array((x,y))- np.array((x_prev,y_prev))) > 3 * step:
            #             break
            #         segment.append((x,y))
            #         # Check in nearest neighbors if euclidean distance is far smaller than distance inside the route
            #         if (x+step, y) in found_nodes:
            #             num_in_seq = found_nodes.index((x+step,y))
            #             if node_route[num_in_seq] > i+threshold:
            #                 flag = True
            #         if (x-step, y) in found_nodes:
            #             num_in_seq = found_nodes.index((x-step,y))
            #             if node_route[num_in_seq] > i+threshold:
            #                 flag = True
            #         if (x, y+step) in found_nodes:
            #             num_in_seq = found_nodes.index((x,y+step))
            #             if node_route[num_in_seq] > i+threshold:
            #                 flag = True
            #         if (x, y-step) in found_nodes:
            #             num_in_seq = found_nodes.index((x,y-step))
            #             if node_route[num_in_seq] > i+threshold:
            #                 flag = True
            #         i += 1
            #     splited_node_route.append(segment)
            # # visualize results
            #
            # # Find segments with 1 or 2 points and insert them in closest segment
            # list_to_pop = []
            # for i in range(len(splited_node_route)):
            #     segment = splited_node_route[i]
            #     if len(segment) <= 2:
            #         point = segment[0]
            #         list_to_pop.append(i)
            #         temp_nodes = found_nodes[:]
            #         for n in segment:
            #             temp_nodes.remove(n)
            #         dist = cdist([point], np.array(temp_nodes), 'euclidean')
            #         min = temp_nodes[dist.argmin()]
            #         for s in splited_node_route:
            #             if min in s:
            #                 s.extend(segment)
            #                 break
            # # Delete these small segments
            # list_to_pop.reverse()
            # for i in list_to_pop:
            #     del splited_node_route[i]
            #
            # # id = self.visualise_node_segments(splited_node_route, id)
            #
            #
            # # Calculate distances of segments
            # start_node = []
            # finish_node = []
            # for segment in splited_node_route:
            #     start_node.append(segment[0])
            #     finish_node.append(segment[-1])
            # segment_dist = cdist(np.array(start_node), np.array(finish_node), 'euclidean')
            # np.fill_diagonal(segment_dist, 0)
            #
            # # Call hill climb algorithm on segments
            # max_iterations = 2000 * len(splited_node_route)
            # # # segment_route, cost, iter = self.routing.anneal(segment_dist, max_iterations, 1.0, 0.9)
            # segment_route, cost, iter = self.routing.random_restart_hillclimb(segment_dist, max_iterations)
            # # segment_route, cost, iter = self.routing.step_hillclimb(segment_dist, max_iterations, step)
            # # segment_route = range(len(splited_node_route))
            #
            # # Reverse segments if make route shorter
            # final_route = []
            # for i in range(len(segment_route)):
            #     segment = splited_node_route[segment_route[i]]
            #     if i == 0 or len(segment) == 1:
            #         final_route.extend(segment)
            #     else:
            #         if np.linalg.norm(np.array(segment[0]) - np.array(final_route[-1])) < \
            #                 np.linalg.norm(np.array(segment[-1]) - np.array(final_route[-1])):
            #             final_route.extend(segment)
            #         else:
            #             segment.reverse()
            #             final_route.extend(segment)
            #
            # # Calculate final route length
            # distances = cdist(np.array(final_route), np.array(final_route), 'euclidean')
            # cost = self.routing.route_length(distances, range(len(final_route)))
            # print('Segment HC route length', cost)

            # # Reorder according to step HC results
            # first_route = final_route[:]

            # # Do another hillclimb to optimize path
            # max_iterations = 2000 * len(first_route)
            # # node_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
            # node_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations)
            # print('Final HC route length', cost, 'made iters', iter, 'from', max_iterations)
            #
            # final_route = []
            # for n in node_route:
            #     final_route.append(first_route[n])


            found_nodes_with_yaw = []
            k = 1   # DEBUG:
            for n in range(len(final_route)):
                # print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                # Find closest obstacle to get best yaw
                x, y = final_route[n]
                x2, y2 = final_route[(n+1)%len(final_route)]
                # Find cover_first poses
                yaw = self.find_best_yaw((x,y), (x2,y2), loop_threshold, obstacle_weight, rotation_weight)
                if yaw != []:
                    for point in yaw:
                        temp_dict = {'position': (x,y), 'yaw': point}
                        found_nodes_with_yaw.append(temp_dict)
                k += 1

            # print(found_nodes_with_yaw)
            self.wall_follow_nodes.append(found_nodes)
            self.wall_follow_sequence.append(found_nodes_with_yaw)

            # for i in range(len(found_nodes)):
            #     node = found_nodes[node_route[i]]
            #     self.print_markers([node], [0., 0., 1.], self.room_node_pub)
            #     rospy.sleep(0.5)
            # rospy.sleep(2)


        # Save wall nodes to json
        self.data['wall_follow_best_yaw_weigths'] = {'loop_threshold':loop_threshold, 'obstacle_weight':obstacle_weight, 'rotation_weight':rotation_weight}
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
            data_to_json = json.dump(self.data, outfile)

        return

    # Do an a priori coverage with found order of nodes to eliminate the not needed
    def a_priori_coverage(self, nodes):
        new_wall_follow = []
        i = 0
        for room in nodes:
            # i = 0
            new_room = []
            for node in room:
                x, y = node['position']
                yaw = node['yaw']
                updated = self.coverage.checkAndUpdateCover(self.brush, [x,y,yaw], 0.85)
                if updated:
                    new_room.append(node)

            self.coverage.coverage_pub.publish(self.coverage.coverage_ogm)
            new_wall_follow.append(new_room)
            rospy.loginfo("A-priori coverage done at room {0}, room nodes size changed from {1} to {2}".format(i, len(room), len(new_room)))
            i += 1

        near_obstacles = np.where(self.brush == 2)
        near_obstacles_cover = self.coverage.coverage[near_obstacles]
        covered_obstacles = len(np.where(near_obstacles_cover >= 80)[0])
        rospy.loginfo("Estimated coverage percentage {}".format(covered_obstacles/len(near_obstacles_cover)))

        return new_wall_follow

    # Find nodes for wall following coverage and eliminate unnecessary (NN) onces
    def find_half_wall_nodes(self, save_result):
        rospy.loginfo("Finding wall follow nodes...")
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        # Uniform sampling on map
        nodes, step = self.uniform_sampling()

        self.print_markers(nodes, [1., 0., 0.], self.node_publisher)
        rospy.sleep(2)

        # Find rooms of nodes
        # Fill door holes with "wall"
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)

        for i in range(len(self.rooms)):
            # Brushfire inside each room and gather brushed wall_follow_nodes
            found_nodes = []
            brush = self.brushfire_cffi.pointBrushfireCffi(self.rooms[i], filled_ogm)
            for n in nodes:
                if brush[n] > 0:
                    found_nodes.append(n)
            found_nodes.sort()  # Optimize path finding
            print("Found {} nodes in room {}.".format(len(found_nodes), i))  # # DEBUG

            nodes_length = len(found_nodes)
            distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
            print('Distances of nodes done.')    # DEBUG:

            # Call hill climb algorithm
            max_iterations = 150 * nodes_length
            # node_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
            node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, step)
            print('Route of wall follow nodes found.')

            found_nodes_with_yaw = []
            k = 1   # DEBUG:
            eliminate = True
            full_found_nodes = found_nodes
            found_nodes = []
            for n in range(len(full_found_nodes)):
                print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                # Find closest obstacle to get best yaw
                x, y = full_found_nodes[node_route[n]]
                # Eliminate unnecessary nodes
                if eliminate:
                    if n == 0:
                        pass
                    elif n == len(full_found_nodes) - 1:
                        pass
                    else:
                        x_prev, y_prev = full_found_nodes[node_route[n-1]]
                        x_next, y_next = full_found_nodes[node_route[n+1]]
                        if np.abs(x-x_prev) == step and np.abs(x-x_next) == step and y == y_prev and y == y_next \
                                or np.abs(y-y_prev) == step and np.abs(y-y_next) == step and x == x_prev and x == x_next:
                            eliminate = False
                            continue
                else:
                    eliminate = True
                found_nodes.append((x,y))
                obstacles = self.brushfire_cffi.closestObstacleBrushfireCffi((x,y), self.ogm)
                for point in obstacles:
                    # Calculate yaw for each obstacle
                    x_diff = point[0] - x
                    y_diff = point[1] - y
                    yaw = math.atan2(y_diff, x_diff)

                    # Add dictionary to right room
                    # # TODO: find best sensor directon on top of yaw
                    dir = self.sensor_direction[0]
                    temp_dict = {'position': (x,y), 'yaw': yaw - dir}
                    found_nodes_with_yaw.append(temp_dict)
                k += 1
            # print(found_nodes_with_yaw)
            self.wall_follow_nodes.append(found_nodes)
            self.wall_follow_sequence.append(found_nodes_with_yaw)

            # for i in range(len(found_nodes)):
            #     node = found_nodes[node_route[i]]
            #     self.print_markers([node], [0., 0., 1.], self.room_node_pub)
            #     rospy.sleep(0.5)
            # rospy.sleep(2)


        if save_result:
            # Save wall nodes to json
            self.data['wall_follow_nodes'] = self.wall_follow_nodes
            self.data['wall_follow_sequence'] = self.wall_follow_sequence
            # self.data['boustrophedon_sequence'] = self.boustrophedon_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

        return

    # Find nodes for wall following coverage and eliminate doubles
    def find_half_no_double_wall_nodes(self, save_result):
        rospy.loginfo("Finding wall follow nodes...")
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        # Uniform sampling on map
        nodes, step = self.uniform_sampling()

        self.print_markers(nodes, [1., 0., 0.], self.node_publisher)
        rospy.sleep(2)

        # Find rooms of nodes
        # Fill door holes with "wall"
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)

        for i in range(len(self.rooms)):
            # Brushfire inside each room and gather brushed wall_follow_nodes
            found_nodes = []
            brush = self.brushfire_cffi.pointBrushfireCffi(self.rooms[i], filled_ogm)
            for n in nodes:
                if brush[n] > 0:
                    found_nodes.append(n)
            found_nodes.sort()  # Optimize path finding
            print("Found {} nodes in room {}.".format(len(found_nodes), i))  # # DEBUG

            nodes_length = len(found_nodes)
            distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
            print('Distances of nodes done.')    # DEBUG:

            # Call hill climb algorithm
            max_iterations = 150 * nodes_length
            # node_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
            node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, step)
            print('Route of wall follow nodes found.')

            found_nodes_with_yaw = []
            k = 1   # DEBUG:
            eliminate = True
            full_found_nodes = found_nodes
            found_nodes = []
            all_obstacles = []
            for i in range(len(full_found_nodes)):
                print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                # Find closest obstacle to get best yaw
                # Eliminate unnecessary nodes
                x, y = full_found_nodes[node_route[i]]
                if eliminate:
                    if i == 0:
                        pass
                    elif i == len(full_found_nodes) - 1:
                        pass
                    else:
                        x_prev, y_prev = full_found_nodes[node_route[i-1]]
                        x_next, y_next = full_found_nodes[node_route[i+1]]
                        if np.abs(x-x_prev) == step and np.abs(x-x_next) == step and y == y_prev and y == y_next \
                                or np.abs(y-y_prev) == step and np.abs(y-y_next) == step and x == x_prev and x == x_next:
                            eliminate = False
                            continue
                else:
                    eliminate = True

                obstacles = self.brushfire_cffi.closestObstacleBrushfireCffi((x,y), self.ogm)

                for ob in obstacles:
                    found_nodes.append((x,y))
                    all_obstacles.append(ob)
                k += 1

            # # TODO: find better epsilon
            epsilon = min(self.sensor_range)/(self.resolution * 10)
            # Find distances between closest obstacles of nodes
            obstacle_dist = cdist(np.array(all_obstacles), np.array(all_obstacles), 'euclidean')
            indexes = zip(*np.where(obstacle_dist <= epsilon))

            # TODO: make faster implementation of elimination
            eliminate = []
            for x,y in indexes:
                if x != y:
                    if self.brush[found_nodes[x]] > self.brush[found_nodes[y]]:
                        eliminate.append(x)
                    else:
                        eliminate.append(y)

            for i in range(len(all_obstacles)):
                if i in eliminate:
                    eliminate.remove(i)
                    continue
                x,y = found_nodes[i]
                point = all_obstacles[i]
                # Calculate yaw for each obstacle
                x_diff = point[0] - x
                y_diff = point[1] - y
                yaw = math.atan2(y_diff, x_diff)

                # Add dictionary to right room
                # # TODO: find best sensor directon on top of yaw
                dir = self.sensor_direction[0]
                temp_dict = {'position': (x,y), 'yaw': yaw - dir}
                found_nodes_with_yaw.append(temp_dict)


            # print(found_nodes_with_yaw)
            self.wall_follow_nodes.append(found_nodes)
            self.wall_follow_sequence.append(found_nodes_with_yaw)

            # for i in range(len(found_nodes)):
            #     node = found_nodes[node_route[i]]
            #     self.print_markers([node], [0., 0., 1.], self.room_node_pub)
            #     rospy.sleep(0.5)
            # rospy.sleep(2)


        if save_result:
            # Save wall nodes to json
            self.data['wall_follow_nodes'] = self.wall_follow_nodes
            self.data['wall_follow_sequence'] = self.wall_follow_sequence
            # self.data['boustrophedon_sequence'] = self.boustrophedon_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

        return


    def read_ogm(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        self.ogm_raw = np.array(data.data)
        self.ogm_width = data.info.width
        self.ogm_height = data.info.height
        self.ogm_header = data.header
        self.ogm = np.zeros((data.info.width, data.info.height), \
                dtype = np.int)

        # Reshape ogm to a 2D array
        for x in range(0, data.info.width):
            for y in range(0, data.info.height):
                self.ogm[x][y] = data.data[x + data.info.width * y]

        # Initilize coverage OGM with same size width x height
        self.coverage.coverage = np.zeros((self.ogm_width, self.ogm_height))
        self.coverage.coverage_ogm.info = data.info
        self.coverage.coverage_ogm.data = np.zeros(self.ogm_width * self.ogm_height)
        self.coverage.ogm = self.ogm
        self.coverage.ogm_raw = self.ogm_raw
        self.coverage.ogm_width = self.ogm_width
        self.coverage.ogm_height = self.ogm_height
        self.coverage.ogm_header = self.ogm_header

        self.ogm_compute = False
        return

    # Method to publish points to rviz
    def print_markers(self, nodes, color, publisher, id = 0):

        points = []
        for point in nodes:
            p = Point()
            p.x = point[0] * self.resolution + self.origin['x']
            p.y = point[1] * self.resolution + self.origin['y']
            p.z = 0
            points.append(p)

        # Create Marker for nodes
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = id
        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.points = points
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        # rospy.loginfo("Printing nodes!")
        publisher.publish(marker)
        return




if __name__ == '__main__':
    node = Map_To_Graph()
    node.server_start()
