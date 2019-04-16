#!/usr/bin/env python
import rospy, json
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from brushfire import Brushfire
from utilities import Cffi
from topology import Topology

class Map_To_Graph:

    def __init__(self):
        pass




if __name__ == '__main__':
    node = Map_To_Graph()
