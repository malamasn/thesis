#!/usr/bin/env python
import rospy, tf, time
import numpy as np
from scipy.spatial import distance

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

        # Read sensor's specs
        self.sensor_name = rospy.get_param('sensor_name')
        self.sensor_direction = rospy.get_param('sensor_direction')
        self.sensor_fov = rospy.get_param('fov')
        self.sensor_range = rospy.get_param('range')
        self.sensor_shape = rospy.get_param('shape')
        self.sensor_reliability = rospy.get_param('reliability')

        # Load map's translation
        translation = rospy.get_param('origin')
        self.origin['x'] = translation[0]
        self.origin['y'] = translation[1]
        self.resolution = rospy.get_param('resolution')

        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = []
        # coverage_ogm will be published in the coverage_topic
        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = "map"
        self.coverage_topic = '/map/coverage'

        self.current_pose = Pose()
        # robot_pose is current_pose in map's frame (aka in pixels)
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0


        self.coverage_pub = rospy.Publisher(self.coverage_topic, \
            OccupancyGrid, queue_size = 10)



    def server_start(self):
        rospy.init_node('coverage')
        rospy.loginfo('Coverage node initialized.')

        # Read ogm
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        rospy.loginfo("Waiting 5 secs to read ogm.")
        time.sleep(5)
        while not rospy.is_shutdown():
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
            rospy.sleep(0.1)
            self.updateCover()
            rospy.sleep(0.9)
            # rospy.Timer(rospy.Duration(0.2), self.readRobotPose)
            # rospy.spin()
        return

    # def readRobotPose(self, event):
    #     rospy.Subscriber('/odom', Odometry, self.odom_callback)
    #     # time.sleep(1)
    #     return

    def updateCover(self):
        cover_length = int(self.sensor_range / self.resolution)
        xx = self.robot_pose['x']
        yy = self.robot_pose['y']
        th = self.robot_pose['th']
        # print(self.robot_pose, 'xx,yy', xx, yy)
        # # cover_length = 20
        # updates = 0
        if self.sensor_shape == 'rectangular':
            for i in range(-cover_length, cover_length):
                for j in range(-cover_length, cover_length):
                    x = int(xx + i)
                    y = int(yy + j)
                    if x < 0 or y < 0 or x >= self.ogm_width or y >= self.ogm_height:
                        continue
                    if self.ogm[x, y] > 49 or self.ogm[x, y] == -1:
                        continue
                    # Check if point inside fov of sensor
                    # cosine = distance.cosine([xx, x], [yy, y])
                    # angle = np.arccos(cosine)
                    # if angle - self.sensor_direction > self.sensor_fov / 2 or \
                    #         angle - self.sensor_direction < -self.sensor_fov / 2:
                    #     continue
                    self.coverage[x, y] = 100
                    index = int(x + self.ogm_width * y)
                    self.coverage_ogm.data[index] = 100
                    # updates += 1
        elif self.sensor_shape == 'circular':

            for i in range(-cover_length, cover_length):
                for j in range(-cover_length, cover_length):
                    x = int(xx + i)
                    y = int(yy + j)
                    if x < 0 or y < 0 or x >= self.ogm_width or y >= self.ogm_height:
                        continue
                    if self.ogm[x, y] > 49 or self.ogm[x, y] == -1:
                        continue
                    # Check if point inside cover circle
                    if distance.euclidean([xx, yy], [x, y]) >= cover_length:
                        continue
                    # # Check if point inside fov of sensor
                    # cosine = distance.cosine([xx, x], [yy, y])
                    # angle = np.arccos(cosine)
                    # if angle - self.sensor_direction > self.sensor_fov / 2 or \
                    #         angle - self.sensor_direction < -self.sensor_fov / 2:
                    #     continue
                    self.coverage[x][y] = 100
                    index = int(x + self.ogm_width * y)
                    self.coverage_ogm.data[index] = 100
                    # updates += 1


        # print('updates' , updates)
        self.coverage_pub.publish(self.coverage_ogm)
        return


    # Cb to read robot's pose
    def odom_callback(self, msg):
        self.current_pose =  msg.pose.pose

        # Calculate current x,y in map's frame (aka in pixels)
        self.robot_pose['x'] = (self.current_pose.position.x - self.origin['x'])/self.resolution
        self.robot_pose['y'] = (self.current_pose.position.y - self.origin['y'])/self.resolution

        # Getting the Euler angles
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        angles = tf.transformations.euler_from_quaternion(q)
        self.robot_pose['th'] = angles[2]   # yaw
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

        # Initilize coverage OGM with same size width x height
        self.coverage = np.zeros((self.ogm_width, self.ogm_height))
        self.coverage_ogm.info = data.info
        self.coverage_ogm.data = np.zeros(self.ogm_width * self.ogm_height)

        return



if __name__ == '__main__':
    node = Coverage()
    node.server_start()
