#!/usr/bin/env python
import rospy, json, actionlib
import numpy as np
import time, tf
from dijkstar import Graph, find_path
from operator import itemgetter

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from utilities import Cffi
from topology import Topology
from coverage import Coverage

class Navigation:

    def __init__(self):
        # self.routing = Routing()
        self.brushfire_cffi = Cffi()
        self.topology = Topology()

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.resolution = 0

        # Load map's translation
        translation = rospy.get_param('origin')
        self.origin['x'] = translation[0]
        self.origin['y'] = translation[1]
        self.resolution = rospy.get_param('resolution')

        # Flag to wait ogm subscriber to finish
        self.ogm_compute = False

        self.current_pose = Pose()

        # OGM related attributes
        self.ogm_topic = '/map'
        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0

        self.gvd = 0
        self.brush = 0

        # Load path pattern for navigation
        self.navigation_pattern = rospy.get_param('navigation_pattern')

        # Attributes read from json file
        self.nodes = []
        self.door_nodes = []
        self.rooms = []
        self.room_doors = []
        self.room_type = []
        self.room_sequence = []
        self.wall_follow_nodes = []

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
        self.wall_follow_nodes = data['wall_follow_nodes']

        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


    # Cb to read robot's pose
    def odom_callback(self, msg):
        self.current_pose =  msg.pose.pose
        return

    def server_start(self):
        rospy.init_node('navigation')
        rospy.loginfo('navigation node initialized.')

        # Read ogm
        self.ogm_compute = True
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.read_ogm)
        while self.ogm_compute:
            pass

        # Calculate current x,y in map's frame
        current_x = (self.current_pose.position.x - self.origin['x'])/self.resolution
        current_y = (self.current_pose.position.y - self.origin['y'])/self.resolution

        # Calculate brushfire field
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)

        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.ogm, self.brush)

        # Find current room
        start = (int(current_x), int(current_y))
        if self.gvd[start]:
            gvd_nodes = [start]
        else:
            gvd_nodes = self.brushfire_cffi.pointToGvdBrushfireCffi(start, self.ogm, self.gvd)
        if not len(gvd_nodes):
            rospy.loginfo("Error. No gvd in the neighborhood.")
            return
        neighbor_nodes = self.brushfire_cffi.gvdNeighborBrushfireCffi(gvd_nodes[0], self.nodes, self.gvd)

        current_room = -1
        for p in neighbor_nodes:
            node = list(p)
            if node in self.door_nodes:
                continue
            for i in range(len(self.rooms)):
                if node in self.rooms[i]:
                    current_room = i
                    break
            if current_room != -1:
                break
        if current_room == -1:
            rospy.loginfo("Problem finding current room! Exiting...")
            return


        current_room_index = self.room_sequence.index(current_room)

        if self.navigation_pattern == 'random':
            self.print_markers(self.nodes)
            # Navigate in all rooms with given sequence
            for i in range(len(self.room_sequence)):
                # find nodes of current room
                # # TO DO: USE ENHANCED NODES
                nodes = self.rooms[current_room]
                # # TO DO: ADD SEQUENCE PATTERN
                nodes.sort()    # for debugging to be faster
                # # TODO: ADD ORIENATION
                # navigate to all nodes
                for node in nodes:
                    result = self.goToGoal(node, 0)
                    rospy.sleep(0.1)
                # TODO: KEEP TRACK OF VISITED NODES (should I?)
                current_room_index = (current_room_index + 1) % len(self.room_sequence)
                current_room = self.room_sequence[current_room_index]

        elif self.navigation_pattern == 'wall_follow':

            for i in range(len(self.room_sequence)):
                # print nodes
                nodes = [k['position'] for k in self.wall_follow_nodes[current_room]]
                self.print_markers(nodes)
                # find nodes of current room
                nodes = self.wall_follow_nodes[current_room]
                # navigate to all nodes
                for node in nodes:
                    result = self.goToGoal(node['position'], node['yaw']-self.sensor_direction)
                    rospy.sleep(0.1)
                current_room_index = (current_room_index + 1) % len(self.room_sequence)
                current_room = self.room_sequence[current_room_index]

        elif self.navigation_pattern == 'boustrophedon':
            for i in range(len(self.room_sequence)):
                # print nodes
                nodes = [k['position'] for k in self.wall_follow_nodes[current_room]]
                self.print_markers(nodes)
                # find nodes of current room
                nodes = sorted(self.wall_follow_nodes[current_room], key=itemgetter('position'))
                # navigate to all nodes
                for node in nodes:
                    result = self.goToGoal(node['position'], node['yaw']-self.sensor_direction)
                    rospy.sleep(0.1)
                current_room_index = (current_room_index + 1) % len(self.room_sequence)
                current_room = self.room_sequence[current_room_index]


        return

    # Gets target pose, sends it to move_base and waits for the result
    def goToGoal(self, target, yaw):
        self.move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target[0]*self.resolution + self.origin['x']
        goal.target_pose.pose.position.y = target[1]*self.resolution + self.origin['y']
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()


    def read_ogm(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        rospy.loginfo("Navigation node reading ogm.")
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

        rospy.loginfo("OGM read!")
        self.ogm_compute = False
        return

    def print_markers(self, nodes):

        rospy.loginfo("Start collecting markers")
        points = []
        for point in nodes:
            p = Point()
            p.x = point[0] * self.resolution + self.origin['x']
            p.y = point[1] * self.resolution + self.origin['y']
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
        return

if __name__ == '__main__':
    node = Navigation()
    node.server_start()
