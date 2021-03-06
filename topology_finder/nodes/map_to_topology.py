#!/usr/bin/env python
import rospy, json
import numpy as np
import time

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from utilities import Cffi
from topology import Topology
from PIL import Image


class Map_To_Topology:

    def __init__(self):
        # self.brushfire = Brushfire()
        self.brushfire_cffi = Cffi()
        self.topology = Topology()

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.resolution = 0.05

        self.ogm = 0
        self.ogm_raw = 0
        self.ogm_width = 0
        self.ogm_height = 0
        self.ogm_header = 0
        self.gvd = 0
        self.brush = 0
        self.nodes = []
        self.door_nodes = []
        # self.brush_publisher = rospy.Publisher('/brushfire', OccupancyGrid, queue_size = 10)
        self.node_publisher = rospy.Publisher('/nodes', Marker, queue_size = 100)
        self.candidate_door_node_pub = rospy.Publisher('/nodes/candidateDoors', Marker, queue_size = 100)
        self.door_node_pub = rospy.Publisher('/nodes/doors', Marker, queue_size = 100)
        self.room_node_pub = rospy.Publisher('/nodes/rooms', Marker, queue_size = 100)

        # Load nodes from json file
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'r') as read_file:
            self.data = json.load(read_file)

        if 'nodes' in self.data:
            self.nodes = self.data['nodes']


    def server_start(self):
        rospy.init_node('map_to_topology')
        rospy.loginfo('map_to_topology node initialized.')

        if rospy.has_param('origin'):
            translation = rospy.get_param('origin')
            self.origin['x'] = translation[0]
            self.origin['y'] = translation[1]
        if rospy.has_param('resolution'):
            self.resolution = rospy.get_param('resolution')

        ogm_topic = '/map'
        rospy.Subscriber(ogm_topic, OccupancyGrid, self.read_ogm)
        rospy.loginfo("Waiting 5 secs to read ogm.")
        time.sleep(5)
        rospy.loginfo("5 secs passed.")

        # Calculate brushfire field
        rospy.loginfo("Brushfire initialized.")
        self.brush = self.brushfire_cffi.obstacleBrushfireCffi(self.ogm)
        rospy.loginfo("Brushfire done!")
        # img = Image.fromarray(self.brush)
        # img.show()
        # img = img.convert('RGB')
        # img.save('indoors_with_rooms_brushfire.png')

        # Calculate gvd from brushfire and ogm
        self.gvd = self.topology.gvd(self.brush)
        # show and/or save gvd as image file
        # img2 = Image.fromarray(255*self.gvd)
        # img2.show()
        # img2 = img2.convert('RGB')
        # img2.save("indoors_with_rooms_gvd.png")

        # Calculate topological nodes
        if self.nodes == []:
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

        self.print_markers(self.nodes, [1.,0.,0.], self.node_publisher)



        # Create list of candidate nodes
        candidateDoors = []
        for point in self.nodes:
            x = point[0]
            y = point[1]
            count_neighbors = np.sum(self.gvd[x-1:x+2, y-1:y+2])
            if count_neighbors == 3:
                candidateDoors.append((x,y))

        if self.door_nodes == []:
            # Send candidateDoors to rviz as Point() values with different color
            self.print_markers(candidateDoors, [0.,1.,0.], self.candidate_door_node_pub)

            # print(candidateDoors)
            # for i in candidateDoors:
            #     print(self.brush[i])
            # Calculate door nodes
            self.door_nodes = self.topology.findDoorNodes(candidateDoors,\
                            self.ogm, self.gvd, self.brushfire_cffi)
            # door_nodes = candidateDoors
            # print(door_nodes)
            # Send door nodes to rviz with different color
            self.print_markers(self.door_nodes, [0.,0.,1.], self.door_node_pub)

        rooms, roomDoors, roomType = self.topology.findRooms(\
                self.gvd, self.door_nodes, self.nodes, self.brush, \
                self.ogm, self.resolution, self.brushfire_cffi)
        print('rooms',rooms,'roomDoors', roomDoors,'roomType', roomType)
        #
        #
        # # Keep as nodes only ones that correspond to a room
        # self.nodes = []
        # for room in rooms:
        #     self.nodes.extend(room)
        # self.print_markers(self.nodes, [1.,0.,0.], self.node_publisher)

        # # Save data to json file
        data = {"nodes": self.nodes, "doors": self.door_nodes,
                "rooms": rooms, "roomDoors": roomDoors, "roomType": roomType}
        # data = {"nodes": self.nodes, "doors": self.door_nodes}
        map_name = rospy.get_param('map_name')
        filename = '/home/mal/catkin_ws/src/topology_finder/data/' + map_name +'.json'
        with open(filename, 'w') as outfile:
            data_to_json = json.dump(data, outfile)

        # while not rospy.is_shutdown():
        # points = []
        # i = 0
        # for room in rooms:
        #     self.print_markers(room, [1.,0.,1.], self.room_node_pub)
        #     print('room type: ', roomType[i])
        #     i += 1
        #     time.sleep(3)
        return


    # Method to publish points to rviz
    def print_markers(self, nodes, color, publisher, id = 0):

        points = []
        for point in nodes:
            p = Point()
            p.x = point[0] * self.resolution + self.origin['x']
            p.y = point[1] * self.resolution + self.origin['y']
            p.z = 0
            points.append(p)

        # Create Marker for nodes
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = id
        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.points = points
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        # rospy.loginfo("Printing nodes!")
        publisher.publish(marker)
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
