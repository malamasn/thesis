#!/usr/bin/env python
import rospy, json
import numpy as np
import time
from dijkstar import Graph, find_path

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker


class Navigation:

    def __init__(self):
        # self.routing = Routing()
        self.resolution = 0.05

        self.current_pose = Pose()

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

        


        return



if __name__ == '__main__':
    node = Navigation()
    node.server_start()
