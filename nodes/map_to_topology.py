#!/usr/bin/env python
import rospy
import numpy as np
import time
from brushfire import Brushfire
from topology import Topology
from nav_msgs.msg import OccupancyGrid


class Map_To_Topology:

    def __init__(self):
        self.brushfire = Brushfire()
        self.topology = Topology()
        self.ogm = 0
        self.ogm_raw = 0
        self.gvd = 0
        self.brush = 0
        self.nodes = []
        self.brush_publisher = rospy.Publisher('/brushfire', OccupancyGrid, queue_size = 10)

    def server_start(self):
        rospy.init_node('map_to_topology')
        ogm_topic = '/map'
        rospy.Subscriber(ogm_topic, OccupancyGrid, self.read_ogm)
        time.sleep(10)
        rospy.spin()



    def read_ogm(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        self.ogm_raw = np.array(data.data)
        self.ogm = np.zeros((data.info.width, data.info.height), \
                dtype = np.int)

        # Reshape ogm to a 2D array
        for x in range(0, data.info.width):
            for y in range(0, data.info.height):
                self.ogm[x][y] = data.data[x + data.info.width * y]

        # Calculate brushfire field
        self.brush = self.brushfire.obstacleBrushfire(self.ogm)
        self.brush_publisher.publish(self.brush)
        # Calculate topological nodes
        self.gvd = self.topology.gvd(self.ogm, self.brush)
        self.nodes = self.topology.topologicalNodes(self.ogm, self.brush, self.gvd)



if __name__ == '__main__':
    node = Map_To_Topology()
    node.server_start()
