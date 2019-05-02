#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose



class Coverage:
    def __init__(self):

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.resolution = 0

        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        self.current_pose = Pose()

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.ogm_sub = rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)


    # Cb to read robot's pose
    def odom_callback(self, msg):
        self.current_pose =  msg.pose.pose
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
    node = Coverage()
