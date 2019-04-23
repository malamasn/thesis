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
        self.resolution = 0.05
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
        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)

    def server_start(self):
        rospy.init_node('map_to_topology')
        rospy.loginfo('map_to_topology node initialized.')
        # rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        # rospy.loginfo("Waiting 5 secs to read ogm.")
        # time.sleep(5)
        # rospy.loginfo("5 secs passed.")
        #
        # # Calculate brushfire field
        # rospy.loginfo("Brushfire initialized.")
        # self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)
        # rospy.loginfo("Brushfire done!")
        # # Calculate gvd from brushfire and ogm
        # self.gvd = self.topology.gvd(self.ogm, self.brush)

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

        # Create list of nodes as Point() values
        rospy.loginfo("Start collecting markers")
        points = []
        for point in self.nodes:
            p = Point()
            p.x = point[0] * self.resolution
            p.y = point[1] * self.resolution
            p.z = 0
            points.append(p)
        rospy.loginfo("Markers ready!")

        # # Create Marker for nodes
        # marker = Marker()
        # marker.header.frame_id = "/map"
        # marker.type = marker.POINTS
        # marker.action = marker.ADD
        #
        # marker.points = points
        # marker.pose.orientation.w = 1.0
        #
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.2
        # marker.color.a = 1.0
        # marker.color.r = 1.0
        #
        # rospy.loginfo("Printing nodes!")
        # self.node_publisher.publish(marker)
        #
        #
        # points = []
        # for point in self.door_nodes:
        #     p = Point()
        #     p.x = point[0] * self.resolution
        #     p.y = point[1] * self.resolution
        #     p.z = 0
        #     points.append(p)
        # rospy.loginfo("Markers ready!")
        #
        # # Create Marker for nodes
        # marker = Marker()
        # marker.header.frame_id = "/map"
        # marker.type = marker.POINTS
        # marker.action = marker.ADD
        #
        # marker.points = points
        # marker.pose.orientation.w = 1.0
        #
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.2
        # marker.color.a = 1.0
        # marker.color.b = 1.0
        #
        # rospy.loginfo("Printing door nodes!")
        # self.door_node_pub.publish(marker)

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

        # for i in range(len(self.door_nodes)):
        #     print(graph[i])

        # Calculate graph's distances between all nodes
        doors_length = len(self.door_nodes)
        distances = np.zeros((doors_length, doors_length), dtype=np.float64)
        for i in range(doors_length):
            for j in range(i+1, doors_length):
                _, _, _, dist = find_path(graph, i, j)
                distances[i][j] = dist
                distances[j][i] = distances[i][j]
        # print(distances)

        # Call hill climb
        max_iterations = 500
        # door_route, cost, iter = self.routing.hillclimb(distances, max_iterations)
        # door_route, cost, iter = self.routing.random_restart_hillclimb(distances, max_iterations)
        door_route, cost, iter = self.routing.anneal(distances, max_iterations, 1.0, 0.95)
        # print(door_route, cost, iter)


        # Correspond door route to room route
        route = []
        for door in door_route:
            for i in range(len(self.room_doors)):
                if self.door_nodes[door] in self.room_doors[i] and i not in route:
                    route.append(i)
        print route, self.routing.route_length(distances, door_route)

        # Save room sequence
        data['room_sequence'] = route
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
                data_to_json = json.dump(data, outfile)
        time.sleep(1)
        # i = 0
        # route = data['room_sequence']
        for _ in range(1):
            for room_id in route:
                # print('room', room)
                points = []
                for point in self.rooms[room_id]:
                    p = Point()
                    p.x = point[0] * self.resolution
                    p.y = point[1] * self.resolution
                    p.z = 0
                    points.append(p)
                rospy.loginfo("Markers ready!")
                # print(p)
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
                marker.color.b = 1.0
                marker.color.r = 1.0

                rospy.loginfo("Printing room nodes!")
                self.room_node_pub.publish(marker)
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



if __name__ == '__main__':
    node = Map_To_Graph()
    node.server_start()
