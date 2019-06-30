#!/usr/bin/env python
from __future__ import division
import rospy, time, math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from collections import Counter

from nav_msgs.msg import OccupancyGrid

from utilities import Cffi

class CoverageNumberSubscriber:

    def __init__(self):
        self.brushfire_cffi = Cffi()

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.resolution = 0

        # Initilize sensor's specs
        self.sensor_names = []
        self.sensor_direction = []
        self.sensor_fov = []
        self.sensor_range = []
        self.sensor_shape = []
        self.sensor_reliability = []

        # Read sensor's specs
        self.sensor_names = rospy.get_param('sensor_names')
        self.sensor_number = len(self.sensor_names)
        for name in self.sensor_names:
            self.sensor_direction.append(rospy.get_param(name + '/sensor_direction'))
            self.sensor_fov.append(rospy.get_param(name + '/fov'))
            self.sensor_range.append(rospy.get_param(name + '/range'))
            self.sensor_shape.append(rospy.get_param(name + '/shape'))
            self.sensor_reliability.append(rospy.get_param(name + '/reliability'))

        # Load map's translation
        translation = rospy.get_param('origin')
        self.origin['x'] = translation[0]
        self.origin['y'] = translation[1]
        self.resolution = rospy.get_param('resolution')

        # OGM related attributes
        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        # Flags to wait subscribers to finish reading data
        self.ogm_compute = True
        self.readPose = False

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = []
        self.coverage_number = []
        self.brush = 0
        # coverage_ogm will be published in the coverage_topic
        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = "map"
        self.coverage_topic = '/map/coverage'

        self.coverage_number_ogm = OccupancyGrid()
        self.coverage_number_ogm.header.frame_id = "map"
        self.coverage_number_topic = '/map/coverage_number'


    def server_start(self):
        rospy.init_node('coverage__number_subscriber')
        rospy.loginfo('Coverage Number Subscriber node initialized.')

        # Read ogm
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        while self.ogm_compute:
            pass

        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)


        # Read coverage angles ogm
        rospy.Subscriber(self.coverage__number_topic, OccupancyGrid, self.cov_callback, queue_size = 1)
        rospy.spin()

        return

    def cov_callback(self, data):
        # Reshape ogm to a 2D array
        for x in range(0, self.ogm_width):
            for y in range(0, self.ogm_height):
                self.coverage_number[x][y] = data.data[x + self.ogm_width * y]

        near_obstacles = np.where(self.brush == 2)

        # Read coverage looks at obstacles
        number = self.coverage_number[near_obstacles]
        # Count values and save to a dictionary
        counted = Counter(number)
        print(counted)
        # Covert to dataframe
        df = pd.DataFrame(counted, index = counted.keys())
        df.drop(df.columns[1:], inplace = True)
        # Plot results
        df.plot(kind='bar')
        plt.show()
        return


    def read_ogm(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        rospy.loginfo("Coverage Subscriber node reading ogm.")

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

        # Initilize coverage OGM with same size width x height
        self.coverage = np.zeros((self.ogm_width, self.ogm_height))
        self.coverage_ogm.info = data.info
        self.coverage_ogm.data = np.zeros(self.ogm_width * self.ogm_height)

        self.coverage_number = np.zeros((self.ogm_width, self.ogm_height))

        rospy.loginfo("OGM read!")
        self.ogm_compute = False
        return



if __name__ == '__main__':
    node = CoverageNumberSubscriber()
    node.server_start()
