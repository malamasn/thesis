#!/usr/bin/env python
from __future__ import division
import rospy, tf, time, math
import numpy as np
from scipy.spatial import distance

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

from utilities import Cffi

class Coverage:
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
        # self.number_of_bins = 2 * int(math.ceil(max_fov/5))     # 5 degrees in each bin
        self.bins = []
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
        self.brush = 0
        # coverage_ogm will be published in the coverage_topic
        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = "map"
        self.coverage_topic = '/map/coverage'

        self.coverage_number_ogm = OccupancyGrid()
        self.coverage_number_ogm.header.frame_id = "map"
        self.coverage_number_topic = '/map/coverage_number'

        self.coverage_angles_ogm = OccupancyGrid()
        self.coverage_angles_ogm.header.frame_id = "map"

        self.coverage_angles_topic = '/map/coverage_angles'

        self.current_pose = Pose()
        # robot_pose is current_pose in map's frame (aka in pixels)
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0

        self.previous_robot_pose = {}
        self.previous_robot_pose['x'] = 0
        self.previous_robot_pose['y'] = 0
        self.previous_robot_pose['th'] = 0

        self.coverage_pub = rospy.Publisher(self.coverage_topic, \
            OccupancyGrid, queue_size = 10)
        self.coverage_number_pub = rospy.Publisher(self.coverage_number_topic, \
            OccupancyGrid, queue_size = 10)
        self.coverage_angles_pub = rospy.Publisher(self.coverage_angles_topic, \
            OccupancyGrid, queue_size = 10)


    def getCoverage(self):
        return np.copy(self.coverage)

    def getCoverageOgm(self):
        return self.coverage_ogm

    def initCoverage(self):
        self.coverage = np.zeros((self.ogm_width, self.ogm_height))
        return


    def server_start(self):
        rospy.init_node('coverage')
        rospy.loginfo('Coverage node initialized.')

        # Read ogm
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        while self.ogm_compute:
            pass

        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)
        near_obstacles = np.where(self.brush == 2)

        iter = 0
        while not rospy.is_shutdown():
            self.updateCover(publish = False)
            if not iter % 500:
                self.coverage_pub.publish(self.coverage_ogm)
                self.coverage_number_pub.publish(self.coverage_number_ogm)
                self.coverage_angles_pub.publish(self.coverage_angles_ogm)
                rospy.loginfo("Update coverage ogm!")
                near_obstacles_cover = self.coverage[near_obstacles]
                covered_obstacles = len(np.where(near_obstacles_cover >= 80)[0])
                rospy.loginfo("Estimated coverage percentage {}".format(covered_obstacles/len(near_obstacles_cover)))
                iter = 0
            iter += 1
            rospy.sleep(0.1)
        return

    # Read current pose of robot from odometry
    def readRobotPose(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback, queue_size = 1)
        return


    def updateCover(self, pose = None, publish = True, track_specs = True):
        if pose == None:
            self.readPose = True
            self.readRobotPose()
            while self.readPose:
                pass
            xx = int(self.robot_pose['x'])
            yy = int(self.robot_pose['y'])
            th = self.robot_pose['th']
            th_deg = math.degrees(th)
        else:
            xx = pose[0]
            yy = pose[1]
            th_deg = pose[2]

        # Check if robot moved
        move = ((xx - self.previous_robot_pose['x'])**2 + \
                (yy - self.previous_robot_pose['y'])**2 + \
                (th_deg - self.previous_robot_pose['th'])**2)**(1/2)

        # Update coverage only if robot moved
        if move > 1.0:
            for s in range(self.sensor_number):
                cover_length = int(self.sensor_range[s] / self.resolution)
                if self.sensor_shape[s] == 'rectangular':
                    indexes = Cffi.rectangularBrushfireCoverageCffi((xx,yy), self.ogm, cover_length, self.sensor_fov[s], th_deg, self.sensor_direction[s])
                    for x,y in indexes:
                        self.coverage[x, y] = 100 * self.sensor_reliability[s]
                        i = int(x + self.ogm_width * y)
                        self.coverage_ogm.data[i] = 100
                        if track_specs:
                            self.coverage_number_ogm.data[i] += 1
                            self.coverage_number[x, y] += 1

                elif self.sensor_shape[s] == 'circular':
                    indexes = Cffi.circularRayCastCoverageCffi((xx,yy), self.ogm, cover_length, self.sensor_fov[s], th_deg, self.sensor_direction[s])
                    for x,y in indexes:
                        self.coverage[x, y] = 100 * self.sensor_reliability[s]
                        i = int(x + self.ogm_width * y)
                        self.coverage_ogm.data[i] = 100
                        if track_specs:
                            self.coverage_number[x, y] += 1
                            self.coverage_number_ogm.data[i] += 1

                            yaw_between_nodes = math.degrees(math.atan2(yy-y, xx-x))
                            angle = yaw_between_nodes + th_deg + self.sensor_direction[s]

                            # print('th', th_deg, 'dir', self.sensor_direction[s], 'yaw', yaw_between_nodes, 'angle', angle)
                            if angle > 180:
                                angle -= 360
                            if angle <= -180:
                                angle += 360
                            for a in range(len(self.bins)):
                                if angle >= self.bins[a][0] and angle < self.bins[a][1]:
                                    self.coverage_angles[x][y][a] += 1
                                    ii = int(x + self.ogm_width * y + self.ogm_height * self.ogm_width * a)
                                    self.coverage_angles_ogm.data[ii] += 1
                                    # print(self.bins[a], a, angle)
                                    break
                else:
                    rospy.loginfo("Error!Sensor's shape not found!")
                    return
            if publish:
                self.coverage_pub.publish(self.coverage_ogm)
                if track_specs:
                    self.coverage_number_pub.publish(self.coverage_number_ogm)
                    self.coverage_angles_pub.publish(self.coverage_angles_ogm)
                rospy.loginfo("Update coverage ogm!")

        self.previous_robot_pose['x'] = xx
        self.previous_robot_pose['y'] = yy
        self.previous_robot_pose['th'] = th_deg
        return

    def checkAndUpdateCover(self, brushfire, pose = None, threshold = 1.):
        if pose == None:
            self.readPose = True
            self.readRobotPose()
            while self.readPose:
                pass
            xx = int(self.robot_pose['x'])
            yy = int(self.robot_pose['y'])
            th = self.robot_pose['th']
            th_deg = math.degrees(th)
        else:
            xx = pose[0]
            yy = pose[1]
            th_deg = pose[2]
        indexes = []
        for s in range(self.sensor_number):
            cover_length = int(self.sensor_range[s] / self.resolution)
            if self.sensor_shape[s] == 'rectangular':
                temp =  Cffi.rectangularBrushfireCoverageCffi((xx,yy), self.ogm, cover_length, self.sensor_fov[s], th_deg, self.sensor_direction[s])
                indexes.extend(temp)
            elif self.sensor_shape[s] == 'circular':
                temp = Cffi.circularRayCastCoverageCffi((xx,yy), self.ogm, cover_length, self.sensor_fov[s], th_deg, self.sensor_direction[s])
                indexes.extend(temp)
            else:
                rospy.loginfo("Error!Sensor's shape not found!")
                return
        covered_area = self.coverage[zip(*indexes)]
        brushfire_area = brushfire[zip(*indexes)]
        near_obstacles_len = len(np.where(brushfire_area == 2)[0])
        if near_obstacles_len == 0:
            return False
        old_obstacles_len = len(np.where((brushfire_area == 2) & (covered_area > 80))[0])
        th = old_obstacles_len / near_obstacles_len
        if th < threshold:
            for x,y in indexes:
                self.coverage[x, y] = 100 * self.sensor_reliability[0]
                i = int(x + self.ogm_width * y)
                self.coverage_ogm.data[i] = 100
            updated = True
        else:
            updated = False
        return updated

    def checkNearbyObstacleCover(self, pose):
        xx = pose[0]
        yy = pose[1]
        th_deg = pose[2]
        return_obstacles = True
        indexes = []
        for s in range(self.sensor_number):
            cover_length = int(self.sensor_range[s] / self.resolution)
            if self.sensor_shape[s] == 'circular':
                temp = Cffi.circularRayCastCoverageCffi((xx,yy), self.ogm, cover_length, self.sensor_fov[s], th_deg, self.sensor_direction[s], return_obstacles)
                indexes.extend(temp)
            else:
                rospy.loginfo("Error!Sensor's shape not found!")
                return
        indexes = list(set(indexes))
        return indexes

    # Cb to read robot's pose
    def odom_callback(self, msg):
        self.current_pose =  msg.pose.pose
        # Calculate current x,y in map's frame (aka in pixels)
        self.robot_pose['x'] = int((self.current_pose.position.x - self.origin['x'])/self.resolution)
        self.robot_pose['y'] = int((self.current_pose.position.y - self.origin['y'])/self.resolution)

        # Getting the Euler angles
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        angles = tf.transformations.euler_from_quaternion(q)
        self.robot_pose['th'] = angles[2]   # yaw

        self.readPose = False
        return

    def read_ogm(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        rospy.loginfo("Coverage node reading ogm.")

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

        # Initilize coverage_number OGM with same size width x height
        self.coverage_number = np.zeros((self.ogm_width, self.ogm_height))
        self.coverage_number_ogm.info = data.info
        self.coverage_number_ogm.data = np.zeros(self.ogm_width * self.ogm_height)

        self.coverage_angles = np.zeros((self.ogm_width, self.ogm_height, self.number_of_bins))
        self.coverage_angles_ogm.info.width = self.ogm_width
        self.coverage_angles_ogm.info.height = self.ogm_height * self.number_of_bins
        self.coverage_angles_ogm.data = np.zeros(self.ogm_width * self.ogm_height * self.number_of_bins)

        rospy.loginfo("OGM read!")
        self.ogm_compute = False
        return



if __name__ == '__main__':
    node = Coverage()
    node.server_start()
