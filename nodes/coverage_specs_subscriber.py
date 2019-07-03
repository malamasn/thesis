#!/usr/bin/env python
from __future__ import division
import rospy, time, math
import numpy as np

from nav_msgs.msg import OccupancyGrid

from utilities import Cffi

class CoverageSubscriber:

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

        # Compute bins to keep with which angle a sensor covers each point
        max_fov = max(self.sensor_fov)/2    # compute for one half (symmetric with sensor_direction)
        self.bins = []
        # self.number_of_bins = 2 * int(math.ceil(max_fov/5))     # 5 degrees in each bin
        # for i in range(self.number_of_bins):
        #     down = - self.number_of_bins/2 * 5 + i*5
        #     up = down + 5
        #     self.bins.append((down,up))

        # Compute bins of negative and positive only
        self.number_of_bins = 2
        self.bins.append((-max_fov, 0))
        self.bins.append((0, max_fov))


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
        self.coverage_angles = []
        self.coverage_diverge = []
        self.brush = 0
        # coverage_ogm will be published in the coverage_topic
        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = "map"
        self.coverage_topic = '/map/coverage'

        self.coverage_diverge_ogm = OccupancyGrid()
        self.coverage_diverge_ogm.header.frame_id = "map"
        self.coverage_diverge_topic = '/map/coverage_angles/diverge'

        self.coverage_angles_ogm = OccupancyGrid()
        self.coverage_angles_ogm.header.frame_id = "map"

        self.coverage_angles_topic = '/map/coverage_angles'

        self.coverage_diverge_pub = rospy.Publisher(self.coverage_diverge_topic, \
            OccupancyGrid, queue_size = 1)


    def server_start(self):
        rospy.init_node('coverage_subscriber')
        rospy.loginfo('Coverage Subscriber node initialized.')

        # Read ogm
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        while self.ogm_compute:
            pass

        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)


        # Read coverage angles ogm
        rospy.Subscriber(self.coverage_angles_topic, OccupancyGrid, self.cov_callback, queue_size = 1)
        rospy.spin()

        return

    def cov_callback(self, data):
        # Reshape ogm to a 3D array
        for a in range(self.number_of_bins):
            for x in range(0, self.ogm_width):
                for y in range(0, self.ogm_height):
                    self.coverage_angles[x][y][a] = data.data[x + self.ogm_width * y + self.ogm_width * self.ogm_height * a]

        near_obstacles = np.where(self.brush == 2)
        near_obstacles_looked = []

        for x in range(0, self.ogm_width):
            for y in range(0, self.ogm_height):
                if self.brush[x][y] == 2:
                    left = self.coverage_angles[x][y][0]
                    right = self.coverage_angles[x][y][1]
                    if left + right:
                        near_obstacles_looked.append((x,y))
                        self.coverage_diverge[x][y] = np.abs(left-right)/(left + right)
                        self.coverage_diverge_ogm.data[x + self.ogm_width * y] = int(100 * self.coverage_diverge[x][y])

        # Retrieve metrics
        values = self.coverage_diverge[near_obstacles]
        rospy.loginfo("Coverage Angle Divergence Mean: {}, Std: {}".format(np.mean(values), np.std(values)))

        values = self.coverage_diverge[zip(*near_obstacles_looked)]
        rospy.loginfo("Coverage Angle Divergence only of covered obstacles Mean: {}, Std: {}".format(np.mean(values), np.std(values)))

        self.coverage_diverge_pub.publish(self.coverage_diverge_ogm)
        #
        # for a in range(self.number_of_bins):
        #     temp = self.coverage_angles[:,:,a].copy()
        #     total_looks = sum(temp[near_obstacles])
        #     string = 'Bin: ' + `self.bins[a]` + ' total: ' + `total_looks`
        #     rospy.loginfo(string)


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

        self.coverage_diverge = np.full((self.ogm_width, self.ogm_height), 100)
        self.coverage_diverge_ogm.info = data.info
        self.coverage_diverge_ogm.data = np.full(self.ogm_width * self.ogm_height, 100)

        self.coverage_angles = np.zeros((self.ogm_width, self.ogm_height, self.number_of_bins))

        rospy.loginfo("OGM read!")
        self.ogm_compute = False
        return



if __name__ == '__main__':
    node = CoverageSubscriber()
    node.server_start()
