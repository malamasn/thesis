#!/usr/bin/env python
import rospy
import numpy as np
import time

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from topology import Topology
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
        self.nodes_with_ids = []
        # self.brush_publisher = rospy.Publisher('/brushfire', OccupancyGrid, queue_size = 10)
        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)

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
        # img = Image.fromarray(self.brush)
        # img.show()
        # img = img.convert('RGB')
        # img.save('indoors_with_rooms_brushfire.png')

        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.ogm, self.brush)
        # show and/or save gvd as image file
        # img2 = Image.fromarray(255*self.gvd)
        # img2.show()
        # img2 = img2.convert('RGB')
        # img2.save("indoors_with_rooms_gvd.png")

        # Calculate topological nodes
        self.nodes = self.topology.topologicalNodes(self.ogm, self.brush, self.gvd)
        # show and/or save nodes as image file
        # temp = np.zeros(self.gvd.shape)
        # for x,y in self.nodes:
        #     temp[x][y] = 255
        # img3 = Image.fromarray(temp)
        # # img3.show()
        # img3 = img3.convert('RGB')
        # img3.save("indoors_with_rooms_nodes.png")

        # Give every node an ID number
        # self.nodes_with_ids[0] has the (x,y) and self.nodes_with_ids[1] the ID
        for i in range(len(self.nodes)):
            self.nodes_with_ids.append((self.nodes[i], i))

        # Create list of nodes as Point() values
        rospy.loginfo("Start collecting markers")
        points = []
        for point in self.nodes:
            p = Point()
            p.x = point[0] * 0.05
            p.y = point[1] * 0.05
            p.z = 0
            points.append(p)
        rospy.loginfo("Markers ready!")

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
        marker.color.r = 1.0

        rospy.loginfo("Printing nodes!")
        self.node_publisher.publish(marker)



        # Create list of candidate nodes
        candidateDoors = []
        for point in self.nodes:
            x = point[0]
            y = point[1]
            count_neighbors = np.sum(self.gvd[x-1:x+2, y-1:y+2])
            if count_neighbors == 3:
                candidateDoors.append((x,y))

        # Send candidateDoors to rviz as Point() values with different color
        points = []
        for point in candidateDoors:
            p = Point()
            p.x = point[0] * 0.05
            p.y = point[1] * 0.05
            p.z = 0
            points.append(p)
        rospy.loginfo("Printing candidate door nodes!")

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
        marker.color.g = 1.0

        rospy.loginfo("Printing markers!")
        self.candidate_door_node_pub.publish(marker)
        # print(candidateDoors)
        # for i in candidateDoors:
        #     print(self.brush[i])
        # Calculate door nodes
        door_nodes = self.topology.findDoorNodes(candidateDoors,\
                        self.nodes, self.gvd, self.brush)
        # print(door_nodes)
        # Send door nodes to rviz with different color
        points = []
        for point in door_nodes:
            p = Point()
            p.x = point[0] * 0.05
            p.y = point[1] * 0.05
            p.z = 0
            points.append(p)
        rospy.loginfo("Printing candidate door nodes!")

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

        rospy.loginfo("Printing door nodes!")
        self.door_node_pub.publish(marker)

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
