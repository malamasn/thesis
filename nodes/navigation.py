#!/usr/bin/env python
import rospy, json
import numpy as np
import time
from dijkstar import Graph, find_path

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

from utilities import Cffi
from topology import Topology

class Navigation:

    def __init__(self):
        # self.routing = Routing()
        self.brushfire_cffi = Cffi()
        self.topology = Topology()

        self.resolution = 0
        self.current_pose = Pose()

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

        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)


    # Cb to read robot's pose
    def odom_callback(self, msg):
        self.current_pose =  msg.pose.pose
        return

    def server_start(self):
        rospy.init_node('navigation')
        rospy.loginfo('navigation node initialized.')

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
        self.room_sequence = data['room_sequence']

        # Load map's translation
        origin = rospy.get_param('origin')
        self.resolution = rospy.get_param('resolution')
        # Calculate current x,y in map's frame
        current_x = (self.current_pose.position.x - origin[0])/self.resolution
        current_y = (self.current_pose.position.y - origin[1])/self.resolution

        # Read ogm
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        rospy.loginfo("Waiting 5 secs to read ogm.")
        time.sleep(5)
        rospy.loginfo("5 secs passed.")
        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)
        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.ogm, self.brush)


        # Find current room
        start = (int(current_x), int(current_y))
        gvd_nodes = self.brushfire_cffi.pointToGvdBrushfireCffi(start, self.ogm, self.gvd)
        neighbor_nodes = self.brushfire_cffi.gvdNeighborBrushfireCffi(gvd_nodes[0], self.nodes, self.gvd)

        current_room = -1
        for p in neighbor_nodes:
            node = list(p)
            if node in self.door_nodes:
                continue
            for i in range(len(self.rooms)):
                if node in self.rooms[i]:
                    current_room = i
                    break
            if current_room != -1:
                break
        if current_room == -1:
            rospy.loginfo("Problem finding current room! Exiting...")
            return

        current_room_index = self.room_sequence.index(current_room)

        for i in range(len(self.room_sequence)):
            # TO DO: NAVIGATE IN THE ROOM


            current_room_index = (current_room_index + 1) % len(self.room_sequence)
            current_room = self.room_sequence[current_room_index]



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
    node = Navigation()
    node.server_start()
