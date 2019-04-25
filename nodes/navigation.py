#!/usr/bin/env python
import rospy, json
import numpy as np
import time
from dijkstar import Graph, find_path




class Navigation:

    def __init__(self):
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
        self.room_sequence = []
        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)


    def server_start(self):
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


        return



if __name__ == '__main__':
    node = Navigation()
    node.server_start()
