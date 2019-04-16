#!/usr/bin/env python
import rospy, json
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from utilities import Cffi
from topology import Topology

class Map_To_Graph:

    def __init__(self):
        self.brushfire_cffi = Cffi()
        self.topology = Topology()
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
        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)

    def server_start(self):
        rospy.init_node('map_to_topology')
        rospy.loginfo('map_to_topology node initialized.')
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        rospy.loginfo("Waiting 5 secs to read ogm.")
        time.sleep(5)
        rospy.loginfo("5 secs passed.")





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
