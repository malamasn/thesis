#!/usr/bin/env python
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
from PIL import Image


class Map_To_Graph:

    def __init__(self):
        self.brushfire_cffi = Cffi()
        self.topology = Topology()
        self.routing = Routing()

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

        # Read sensor's specs
        self.sensor_name = rospy.get_param('rfid/sensor_name')
        self.sensor_direction = rospy.get_param('rfid/sensor_direction')
        self.sensor_fov = rospy.get_param('rfid/fov')
        self.sensor_range = rospy.get_param('rfid/range')
        self.sensor_shape = rospy.get_param('rfid/shape')
        self.sensor_reliability = rospy.get_param('rfid/reliability')
        self.min_distance = rospy.get_param('min_distance')

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
        self.wall_follow_nodes = []
        self.wall_follow_sequence = []
        self.boustrophedon_sequence = []

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

        if 'wall_follow_nodes' in self.data:
            self.wall_follow_nodes = self.data['wall_follow_nodes']

        if 'wall_follow_sequence' in self.data:
            self.wall_follow_sequence = self.data['wall_follow_sequence']

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
            rospy.loginfo("Finding room sequence...")
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

            # Correspond door route to room route
            for door in door_route:
                for i in range(len(self.room_doors)):
                    if self.door_nodes[door] in self.room_doors[i] and i not in route:
                        self.room_sequence.append(i)

            # Save room sequence
            self.data['room_sequence'] = self.room_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)

        # time.sleep(1)
        # # i = 0
        # # route = self.data['room_sequence']
        # for _ in range(1):
        #     for room_id in self.room_sequence:
        #         self.print_markers(self.rooms[room_id], [1.0, 0., 1.0], self.room_node_pub)
        #         print('room type: ', self.room_type[room_id])
        #         # i += 1
        #         time.sleep(3)

        # Find nodes for wall following coverage
        if self.wall_follow_nodes == []:
            rospy.loginfo("Finding wall follow nodes...")

            # Uniform sampling on map
            nodes = []
            # Sampling step is half the sensor's range
            step = int(self.sensor_range /(self.resolution * 2))
            safety_offset = self.min_distance / self.resolution

            for x in range(0, self.ogm_width, step):
                for y in range(0, self.ogm_height, step):
                    # Node should be close to obstacle, but not too close to avoid collision
                    if self.brush[x][y] > safety_offset and self.brush[x][y] <= 2 * step:
                        nodes.append((x,y))

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
                # # Calculate boustrophedon sequence
                # self.boustrophedon_sequence.append(self.boustrophedon(found_nodes, step))

                if self.wall_follow_sequence != []:
                    continue
                
                nodes_length = len(found_nodes)
                distances = cdist(np.array(found_nodes), np.array(found_nodes), 'euclidean')
                print('Distances of nodes done.')    # DEBUG:

                # Call hill climb algorithm
                max_iterations = 150 * nodes_length
                # node_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
                node_route, cost, iter = self.routing.step_hillclimb(distances, max_iterations, step)
                print('Route of wall follow nodes found.')

                found_nodes_with_yaw = []
                k = 0   # DEBUG:
                for node in found_nodes:
                    print('Closest obstacles process: {}/{}'.format(k, nodes_length))
                    # Find closest obstacle to get best yaw
                    x, y = node
                    obstacles = self.brushfire_cffi.closestObstacleBrushfireCffi((x,y), self.ogm)
                    for point in obstacles:
                        # Calculate yaw for each obstacle
                        x_diff = point[0] - x
                        y_diff = point[1] - y
                        yaw = math.atan2(y_diff, x_diff)

                        # Add dictionary to right room
                        temp_dict = {'position': (x,y), 'yaw': yaw}
                        found_nodes_with_yaw.append(temp_dict)
                    k += 1

                # print(found_nodes_with_yaw)
                self.wall_follow_nodes.append(found_nodes_with_yaw)
                self.wall_follow_sequence.append(node_route)

                for i in range(len(found_nodes)):
                    node = found_nodes[node_route[i]]
                    self.print_markers([node], [0., 0., 1.], self.room_node_pub)
                    rospy.sleep(0.5)
                # rospy.sleep(2)



            # Save wall nodes to json
            self.data['wall_follow_nodes'] = self.wall_follow_nodes
            self.data['wall_follow_sequence'] = self.wall_follow_sequence
            self.data['boustrophedon_sequence'] = self.boustrophedon_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                data_to_json = json.dump(self.data, outfile)


        return

    # def boustrophedon(self, nodes, step):
    #
    #     min_y = min(nodes, key=itemgetter(1))[1]
    #     max_y = max(nodes, key=itemgetter(1))[1]
    #     # in = False
    #     # out = False
    #     reverse = False
    #     final_nodes = []
    #     # in_nodes = []
    #
    #     for y in range(min_y, max_y+step, step):
    #         slice = [item for item in nodes if item[1] == y]
    #         slice.sort()
    #         occupied = np.any(self.ogm[slice[0][0]:slice[-1][0], y] > 49)
    #         if reverse:
    #             slice.reverse()
    #         # out event
    #         if not occupied:
    #             final_nodes.extend(slice)
    #             # if in:
    #                 # Go to previous unvisited
    #         else:
    #             # # if hits obstacle
    #             # in = True
    #             # hit = False
    #             # if reverse:
    #             #     hit_index = len(in_nodes)
    #             # else:
    #             #     hit_index = 0
    #             final_nodes.append(slice[0])
    #             for i in range(len(slice)-1):
    #                 min_x = min(slice[i:i+1], key=itemgetter(0))[0]
    #                 max_x = max(slice[i:i+1], key=itemgetter(0))[0]
    #                 occupied = np.any(self.ogm[min_x:max_x, y] > 49)
    #                 if not occupied: # and not hit:
    #                     final_nodes.append(slice[i+1])
    #                 else:
    #                     break
    #                 # elif not occupied and hit:
    #                 #     pass
    #                 # else:
    #                 #     hit = True
    #
    #
    #         reverse = not reverse
    #     sequence = []
    #     for node in final_nodes:
    #         sequence.append(nodes.index(node))
    #     return sequence

        # size = len(found_nodes)
        # temp = []
        # temp.append(found_nodes[0])
        # for ii in range(size-1):
        #     node_1 = found_nodes[ii]
        #     node_2 = found_nodes[ii+1]
        #     dist = np.linalg.norm(np.array(node_1)- np.array(node_2))
        #     if dist <= step:
        #         temp.append(node_2)
        #         continue
        #     else:
        #         pass



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
        self.ogm_compute = False
        return

    # Method to publish points to rviz
    def print_markers(self, nodes, color, publisher):

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

        rospy.loginfo("Printing nodes!")
        publisher.publish(marker)
        return




if __name__ == '__main__':
    node = Map_To_Graph()
    node.server_start()
