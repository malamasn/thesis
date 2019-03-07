#!/usr/bin/env python
import rospy
import numpy as np
import time
from brushfire import Brushfire
from topology import Topology
from nav_msgs.msg import OccupancyGrid
from PIL import Image


class Map_To_Topology:

    def __init__(self):
        self.brushfire = Brushfire()
        self.topology = Topology()
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0
        self.gvd = 0
        self.brush = 0
        self.nodes = []
        # self.brush_publisher = rospy.Publisher('/brushfire', OccupancyGrid, queue_size = 10)

    def server_start(self):
        rospy.init_node('map_to_topology')
        rospy.loginfo('map_to_topology node initialized.')
        ogm_topic = '/map'
        rospy.Subscriber(ogm_topic, OccupancyGrid, self.read_ogm)
        rospy.loginfo("Waiting 5 secs to read ogm.")
        time.sleep(5)
        rospy.loginfo("5 secs passed.")

        # Calculate brushfire field
        self.brush = self.brushfire.obstacleBrushfire(self.ogm)
        # img = Image.fromarray(5*self.brush)
        # img.show()
        # img = img.convert('RGB')
        # img.save('indoor_with_distance_brushfire.png')

        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.ogm, self.brush)
        # show and/or save gvd as image file
        # img2 = Image.fromarray(255*self.gvd)
        # img2.show()
        # img2 = img2.convert('RGB')
        # img2.save("indoor_with_distance_gvd.png")

        # Calculate topological nodes
        self.nodes = self.topology.topologicalNodes(self.ogm, self.brush, self.gvd)
        # show and/or save nodes as image file
        # temp = np.zeros(self.gvd.shape)
        # for x,y in self.nodes:
        #     temp[x][y] = 255
        # img3 = Image.fromarray(temp)
        # img3.show()
        # img3 = img3.convert('RGB')
        # img3.save("indoor_with_distance_nodes.png")

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
    node = Map_To_Topology()
    node.server_start()
