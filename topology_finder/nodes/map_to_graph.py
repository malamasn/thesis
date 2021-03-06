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
        self.resolution = 0.05

        self.step = 0

        # Load map's translation
        if rospy.has_param('origin'):
            translation = rospy.get_param('origin')
            self.origin['x'] = translation[0]
            self.origin['y'] = translation[1]
        if rospy.has_param('resolution'):
            self.resolution = rospy.get_param('resolution')


        # Initilize sensor's specs
        self.sensor_names = []
        self.sensor_direction = []
        self.sensor_fov = []
        self.sensor_range = []
        self.sensor_shape = []
        self.sensor_reliability = []
        self.min_distance = 0
        self.navigation_target = 0

        # Read sensor's specs
        self.robot_radius = rospy.get_param('robot_radius')
        self.sensor_names = rospy.get_param('sensor_names')
        self.sensor_number = len(self.sensor_names)
        self.navigation_target = rospy.get_param('navigation_target')
        self.navigation_pattern = rospy.get_param('navigation_pattern')
        self.fixed_step = rospy.get_param('fixed_step')
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
        self.zig_zag_nodes = []
        self.zig_zag_sequence = []
        self.simple_nodes = []
        self.simple_sequence = []


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
        if 'entering_doors' in self.data:
            self.entering_doors = { int(k):v for k,v in self.data['entering_doors'].items() }
        if 'exiting_doors' in self.data:
            self.exiting_doors = { int(k):v for k,v in self.data['exiting_doors'].items() }

        if 'wall_follow_nodes' in self.data:
            self.wall_follow_nodes = self.data['wall_follow_nodes']

        if 'wall_follow_sequence' in self.data:
            self.wall_follow_sequence = self.data['wall_follow_sequence']

        if 'zig_zag_nodes' in self.data:
            self.zig_zag_nodes = self.data['zig_zag_nodes']

        if 'zig_zag_sequence' in self.data:
            self.zig_zag_sequence = self.data['zig_zag_sequence']

        if 'simple_nodes' in self.data:
            self.simple_nodes = self.data['simple_nodes']

        if 'simple_sequence' in self.data:
            self.simple_sequence = self.data['simple_sequence']


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
        self.gvd = self.topology.gvd(self.brush)

        time.sleep(1)
        # self.print_markers(self.nodes, [1., 0., 0.], self.node_publisher)
        # self.print_markers(self.door_nodes, [0., 0., 1.], self.door_node_pub)

        # Find sequence of rooms if not already exists
        if self.room_sequence == []:
            self.find_room_sequence(True)

        if self.navigation_pattern == 'simple':
            self.find_all_wall_nodes()

            rospy.loginfo("Visualize node sequence")
            self.visualise_node_sequence(self.simple_sequence)
            self.simple_sequence, self.simple_nodes = self.a_priori_coverage(self.simple_sequence, False)

            # Compute length of sequence
            i = 0
            for temp_nodes in self.simple_nodes:
                distances = cdist(np.array(temp_nodes), np.array(temp_nodes), 'euclidean')
                cost = self.routing.route_length(distances, range(len(temp_nodes)))
                string = 'Length of zig zag nodes at room ' + str(i) + ': ' + str(cost)
                print(string)
                i += 1

        else:
            self.find_best_path_wall_nodes()

            self.visualise_node_sequence(self.wall_follow_sequence)
            self.wall_follow_sequence, self.wall_follow_nodes = self.a_priori_coverage(self.wall_follow_sequence)

            # Save wall nodes to json
            self.data['wall_follow_nodes'] = self.wall_follow_nodes
            self.data['wall_follow_sequence'] = self.wall_follow_sequence

            rospy.loginfo("Visualize node sequence")
            self.visualise_node_sequence(self.wall_follow_sequence)

            # Compute length of sequence
            i = 0
            for temp_nodes in self.wall_follow_nodes:
                distances = cdist(np.array(temp_nodes), np.array(temp_nodes), 'euclidean')
                cost = self.routing.route_length(distances, range(len(temp_nodes)))
                string = 'Length of wall follow nodes at room ' + str(i) + ': ' + str(cost)
                print(string)
                i += 1

            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

            print("5 secs sleep with wall_follow_sequence")
            rospy.sleep(5)

            self.zig_zag_sequence, self.zig_zag_nodes = self.add_zig_zag_nodes(self.wall_follow_sequence)

            self.zig_zag_sequence, self.zig_zag_nodes = self.a_priori_coverage(self.zig_zag_sequence, False)

            # Save zig zag nodes to json
            self.data['zig_zag_nodes'] = self.zig_zag_nodes
            self.data['zig_zag_sequence'] = self.zig_zag_sequence

            rospy.loginfo("Visualize zig zag node sequence")
            self.visualise_node_sequence(self.zig_zag_sequence)

            # Compute length of sequence
            i = 0
            for temp_nodes in self.zig_zag_nodes:
                distances = cdist(np.array(temp_nodes), np.array(temp_nodes), 'euclidean')
                cost = self.routing.route_length(distances, range(len(temp_nodes)))
                string = 'Length of zig zag nodes at room ' + str(i) + ': ' + str(cost)
                print(string)
                i += 1

            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

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
        min_range = int(min(self.sensor_range)/self.resolution)
        max_range = int(max(self.sensor_range)/self.resolution)


        step = int(1.5 * self.robot_radius /self.resolution)

        step_list = []
        while step <= max_range:
            temp_nodes = []
            step_list.append(step)
            indx = np.where((self.brush > step) & (self.brush <= 2*step) & (self.brush <= max_range))
            indx = zip(*indx)
            for x,y in indx:
                if not x % step and not y % step:
                    temp_nodes.append((x,y))
            step *= 2
            nodes.append(temp_nodes)

        obstacles = np.zeros((self.ogm_width, self.ogm_height))
        final_nodes = []
        i = len(nodes) - 1
        while i >= 0:
            temp_nodes = nodes[i]
            # self.print_markers(temp_nodes, [1., 0., 0.], self.node_publisher)
            # rospy.sleep(2)

            # Check if new nodes override previous ones and delete them
            for point in temp_nodes:
                x,y = point
                indexes = self.brushfire_cffi.circularRayCastCoverageCffi(point, self.ogm, max_range, 360, 0, 0, True)
                if not len(indexes):
                    continue

                if i == len(nodes) - 1:
                    obstacles[zip(*indexes)] = 1
                    final_nodes.append(point)
                else:
                    temp = obstacles[zip(*indexes)]
                    p = len(np.where(temp > 0)[0])/len(indexes)
                    if p < 0.9:
                        final_nodes.append((x,y))
                        obstacles[zip(*indexes)] = 1
            i -= 1
        return final_nodes, step_list

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
                        # Find brushfire distance (closest to real dist) between doors
                        dist = self.brushfire_cffi.pointToPointBrushfireCffi(tuple(room[i]), \
                                tuple(room[j]), self.ogm)
                        if dist < 0:
                            rospy.loginfo("Error computing door distance!")
                            return
                        graph.add_edge(node_id_1, node_id_2, dist)
                        graph.add_edge(node_id_2, node_id_1, dist)

        # Calculate graph's distances between all nodes
        doors_length = len(self.door_nodes)
        if doors_length > 2:
            distances = np.zeros((doors_length, doors_length), dtype=np.float64)
            for i in range(doors_length):
                for j in range(i+1, doors_length):
                    _, _, _, dist = find_path(graph, i, j)
                    distances[i][j] = dist
                    distances[j][i] = distances[i][j]

            # Call hill climb algorithm
            max_iterations = 500 * doors_length
            # door_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
            door_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations)

            print('Optimal RRHC room sequence', door_route, self.routing.route_length(distances, door_route))
            # door_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
            # print('Optimal ANNEAL room sequence', door_route, self.routing.route_length(distances, door_route))

            #Sub-optimal / greedy sequence
            greedy_route = []
            greedy_route.append(door_route[0])
            for i in range(1, doors_length):
                #find min
                previous = greedy_route[i-1]


                unvisited = []
                for j in range(doors_length):
                    if j not in greedy_route:
                        unvisited.append(j)

                minimum = distances[previous, unvisited[0]]
                min_idx = unvisited[0]
                for idx in unvisited:
                    if distances[previous][idx] < minimum:
                        minimum = distances[previous][idx]
                        min_idx = idx
                greedy_route.append(min_idx)

            greedy_cost = self.routing.route_length(distances, greedy_route)
            print('Greedy room sequence', greedy_route, greedy_cost)

        elif doors_length == 2:
            door_route = [0, 1]
            _, _, _, dist = find_path(graph, 0, 1)
            print('Only 2 doors with distance ', dist)
        elif self.door_nodes != []:
            door_route = [0]
            print('One door found!')
        else:
            door_route = []
            self.room_sequence = [0]
            print('One room only!')
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
        elif self.door_nodes != []:
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
        while found < loop_threshold * len(obstacles) and len(obstacles):

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

    # Find the yaw that maximizes the covered walls of a given node
    def find_yaw(self, node, next_node, obstacle_weight, rotation_weight):
        yaw_between_nodes = math.degrees(math.atan2(next_node[1]-node[1], next_node[0]-node[0]))
        yaw = []

        steps = int(max(self.sensor_range) / self.resolution)
        node_brush = self.brush[node]
        steps = max(steps, node_brush)
        # Find reachable obstacles
        obstacles = self.brushfire_cffi.inRangeObstacleBrushfireCffi(node, self.ogm, steps, True)

        if len(obstacles) == 0:
            return yaw

        # In case of many obstacles more poses are needed for full coverage
        found = 0
        best_evaluation = 0
        best_yaw = 0
        for angle in range(-180, 180, 10):
            pose = [node[0], node[1], angle]
            all_covered_obstacles = self.coverage.checkNearbyObstacleCover(pose)
            covered_obstacles = 0
            for ob in all_covered_obstacles:
                if ob in obstacles:
                    covered_obstacles += 1
            if not covered_obstacles:
                continue

            # Absolute yaw of current and next node in [0,180]
            next_rotation = np.abs(angle - yaw_between_nodes)
            while next_rotation >= 360:
                next_rotation -= 360
            if next_rotation > 180:
                next_rotation = 360 - next_rotation

            # Evaluate candidate angle
            # Use Motor Schema with covered area and rotation to next node as parameters
            evaluation = obstacle_weight * covered_obstacles/len(obstacles) + rotation_weight * (180 - next_rotation)/180
            if evaluation > best_evaluation:
                best_evaluation = evaluation
                best_yaw = angle

        # Check if obstacle is coverable
        if best_evaluation:
            yaw.append(best_yaw)
        return yaw

    def find_weights(self):
        if self.navigation_target == 'cover_first':
            angles = []
            for s in range(self.sensor_number):
                theta = self.sensor_direction[s]
                while theta < 0:
                    theta += 360
                while theta >= 360:
                    theta -= 360
                if theta > 180:
                    theta = 360 - theta
                if theta > 90:
                    theta = 180 - theta
                angles.append((90-theta)/90)
            obstacle_weight = 1 + np.mean(angles)
            rotation_weight = 2 - np.mean(angles)
        elif self.navigation_target == 'fast_first':
            angles = []
            for s in range(len(self.sensor_number)):
                theta = self.sensor_direction[s]
                fov = self.sensor_fov[s]/2
                while theta < 0:
                    theta += 360
                while theta >= 360:
                    theta -= 360
                if theta > 180:
                    theta = 360 - theta
                if theta > 90:
                    theta = 180 - theta
                theta = min(theta+fov, 90)
                angles.append((90-theta)/90)
            obstacle_weight = 1 + np.mean(angles)
            rotation_weight = 2 - np.mean(angles)
        else:
            obstacle_weight = 1
            rotation_weight = 1

        return obstacle_weight, rotation_weight

    # Add zig zag nodes inside already computed node sequence
    def add_zig_zag_nodes(self, wall_follow_sequence):
        zig_zag_nodes = []
        zig_zag_sequence = []
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)
        j = 0
        for room in wall_follow_sequence:
            temp_nodes = []
            temp_sequence = []
            room_brush = self.brushfire_cffi.pointBrushfireCffi(self.rooms[j], filled_ogm)

            for i in range(len(room)):
                if not i:
                    temp_nodes.append(room[i]['position'])
                    temp_sequence.append(room[i])
                    continue

                node = room[i]['position']
                previous_node = room[i-1]['position']
                x_final, y_final = 0, 0
                if previous_node[0] == node[0] and np.abs(previous_node[1]-node[1]) and np.abs(previous_node[1]-node[1]) <= max(self.sensor_range)/self.resolution:
                    dif = np.abs(previous_node[1]-node[1])
                    x1 = int(node[0] - dif/2)
                    x2 = int(node[0] + dif/2)
                    y = int(np.min([previous_node[1],node[1]]) + dif/2)
                    if self.brush[x1,y] >= self.brush[x2,y]:
                        x_final = x1
                        y_final = y
                        yaw = math.degrees(math.atan2(y-node[1], x1-node[0]))
                    else:
                        x_final = x2
                        y_final = y
                        yaw = math.degrees(math.atan2(y-node[1], x2-node[0]))

                elif previous_node[1] == node[1] and np.abs(previous_node[0]-node[0]) and np.abs(previous_node[0]-node[0]) <= max(self.sensor_range)/self.resolution:
                    dif = np.abs(previous_node[0]-node[0])
                    y1 = int(node[1] - dif/2)
                    y2 = int(node[1] + dif/2)
                    x = int(np.min([previous_node[0],node[0]]) + dif/2)
                    if self.brush[x,y1] >= self.brush[x,y2]:
                        x_final = x
                        y_final = y1
                        yaw = math.degrees(math.atan2(y1-node[1], x-node[0]))
                    else:
                        x_final = x
                        y_final = y2
                        yaw = math.degrees(math.atan2(y2-node[1], x-node[0]))

                if room_brush[x_final][y_final] > 1:
                    temp_nodes.append((x_final,y_final))
                    temp_sequence.append({'position': (x_final,y_final), 'yaw': yaw})

                temp_nodes.append(room[i]['position'])
                temp_sequence.append(room[i])

            zig_zag_nodes.append(temp_nodes)
            zig_zag_sequence.append(temp_sequence)
            j += 1

        return zig_zag_sequence, zig_zag_nodes

    # Find nodes for wall following coverage
    def find_all_wall_nodes(self):
        rospy.loginfo("Finding simple strategy nodes...")
        self.simple_nodes = []
        self.simple_sequence = []
        obstacle_weight, rotation_weight = 1, 1

        # Uniform sampling on map
        nodes = []
        step = int(self.fixed_step / self.resolution)
        indx = np.where((self.brush > step) & (self.brush <= 2*step))
        indx = zip(*indx)
        for x,y in indx:
            if not x % step and not y % step:
                nodes.append((x,y))

        self.print_markers(nodes, [1., 0., 0.], self.node_publisher)
        rospy.sleep(2)

        # Find rooms of nodes
        # Fill door holes with "wall"
        filled_ogm = self.topology.doorClosure(self.door_nodes, self.ogm, self.brushfire_cffi)
        id = 0
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
            if nodes_length:
                distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
                print('Distances of nodes done.')    # DEBUG:

            found_nodes.insert(0, self.entering_doors[i])
            found_nodes.append(self.exiting_doors[i])

            # Call RR hillclimb to optimize path
            max_iterations = 2000 * len(found_nodes)
            distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
            cost = self.routing.route_length(distances, range(len(found_nodes)))
            print('Route length after adding doors', cost)
            # node_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
            fixed_edges = True
            node_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations, fixed_edges)
            print('Second HC route length', cost, 'made iters', iter, 'from', max_iterations)


            found_nodes_with_yaw = []
            k = 1   # DEBUG:

            for n in range(len(found_nodes)):

                print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                # Find closest obstacle to get best yaw
                x, y = found_nodes[node_route[n]]
                x2, y2 = found_nodes[node_route[(n+1)%len(node_route)]]
                yaw = self.find_yaw((x,y), (x2,y2), obstacle_weight, rotation_weight)
                if yaw == []:
                    continue
                for point in yaw:
                    temp_dict = {'position': (x,y), 'yaw': point}
                    found_nodes_with_yaw.append(temp_dict)
                k += 1

            self.simple_nodes.append(found_nodes)
            self.simple_sequence.append(found_nodes_with_yaw)

        # Save wall nodes to json
        self.data['simple_nodes'] = self.simple_nodes
        self.data['simple_sequence'] = self.simple_sequence
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
            data_to_json = json.dump(self.data, outfile)

        return

    # Find nodes for wall following coverage with optimal path
    def find_best_path_wall_nodes(self):
        rospy.loginfo("Finding wall follow nodes...")
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []

        loop_threshold = 0.6
        obstacle_weight, rotation_weight = self.find_weights()

        # Uniform sampling on map
        nodes, self.step = self.uniform_sampling()

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
            node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, self.step)
            print('Route of wall follow nodes found.')
            print('First step HC route length', cost)

            # Reorder according to step HC results
            temp_route = []
            for n in node_route:
                temp_route.append(found_nodes[n])

            # Add entering & exiting doors and rotate already found route
            # so that 2nd point is the closest one to the entering door
            first_route = []
            enter = self.entering_doors[i]
            first_route.append(enter)
            closest_idx = cdist(np.array([enter]), np.array(temp_route)).argmin()
            for ii in range(len(temp_route)):
                first_route.append(temp_route[(ii + closest_idx) % nodes_length])
            first_route.append(self.exiting_doors[i])

            # Do another hillclimb to optimize path
            max_iterations = 2000 * len(first_route)
            distances = cdist(np.array(first_route), np.array(first_route), 'euclidean')
            cost = self.routing.route_length(distances, range(len(first_route)))
            print('Route length after adding doors and rotating sequence', cost)

            fixed_edges = True
            node_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations, fixed_edges)
            print('Second HC route length', cost, 'made iters', iter, 'from', max_iterations)

            final_route = []
            for n in node_route:
                final_route.append(first_route[n])


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

            self.wall_follow_nodes.append(found_nodes)
            self.wall_follow_sequence.append(found_nodes_with_yaw)

        # Save wall nodes to json
        self.data['wall_follow_best_yaw_weigths'] = {'loop_threshold':loop_threshold, 'obstacle_weight':obstacle_weight, 'rotation_weight':rotation_weight}
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
            data_to_json = json.dump(self.data, outfile)

        return

    # Do an a priori coverage with found order of nodes to eliminate the not needed
    def a_priori_coverage(self, nodes, eliminate = True):
        self.coverage.initCoverage()

        new_sequence = []
        new_nodes = []
        for room in nodes:
            # i = 0
            new_room = []
            new_room_nodes = []
            for i in range(len(room)):
                node = room[i]
                x, y = node['position']
                yaw = node['yaw']
                if not i or i == len(room)-1 or not eliminate:
                    self.coverage.updateCover([x,y,yaw], publish = False, track_specs = False)
                    updated = True
                else:
                    updated = self.coverage.checkAndUpdateCover(self.brush, [x,y,yaw], 0.85)
                if updated:
                    new_room.append(node)
                    new_room_nodes.append(node['position'])

            self.coverage.coverage_pub.publish(self.coverage.coverage_ogm)
            new_sequence.append(new_room)
            new_nodes.append(new_room_nodes)
            rospy.loginfo("A-priori coverage done at room {0}, room nodes size changed from {1} to {2}".format(i, len(room), len(new_room)))
            i += 1

        near_obstacles = np.where(self.brush == 2)
        near_obstacles_cover = self.coverage.coverage[near_obstacles]
        covered_obstacles = len(np.where(near_obstacles_cover >= 80)[0])
        rospy.loginfo("Estimated coverage percentage {}".format(covered_obstacles/len(near_obstacles_cover)))

        return new_sequence, new_nodes


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
