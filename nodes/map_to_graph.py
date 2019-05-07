#!/usr/bin/env python
import rospy, json
import numpy as np
import time
from dijkstar import Graph, find_path

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from utilities import Cffi
from topology import Topology
from routing import Routing


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

        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        self.gvd = 0
        self.brush = 0

        self.nodes = []
        self.door_nodes = []
        self.rooms = []
        self.room_doors = []
        self.room_type = []
        self.room_sequence = []

        # Load nodes from json file
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'r') as read_file:
            data = json.load(read_file)

        self.nodes = data['nodes']
        self.door_nodes = data['doors']
        self.rooms = data['rooms']
        self.room_doors = data['roomDoors']
        self.room_type = data['roomType']

        if 'room_sequence' in data:
            self.room_sequence = data['room_sequence']

        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)

    def server_start(self):
        rospy.init_node('map_to_topology')
        rospy.loginfo('map_to_topology node initialized.')

        time.sleep(1)
        self.print_markers(self.nodes, [1., 0., 0.], self.node_publisher)
        self.print_markers(self.door_nodes, [0., 0., 1.], self.door_node_pub)

        # Find sequence of rooms if not already exists
        if self.room_sequence == []:
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
            data['room_sequence'] = self.room_sequence
            map_name = rospy.get_param('map_name')
            filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
            with open(filename, 'w') as outfile:
                    data_to_json = json.dump(data, outfile)

        time.sleep(1)
        # i = 0
        # route = data['room_sequence']
        for _ in range(1):
            for room_id in self.room_sequence:
                self.print_markers(self.rooms[room_id], [1.0, 0., 1.0], self.room_node_pub)
                print('room type: ', self.room_type[room_id])
                # i += 1
                time.sleep(3)

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
