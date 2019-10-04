ROS package to extract topology features from a given map.

## Nodes
* **map_to_topology** node read an ogm and produces a topological graph, detects door nodes and implements room segmentation.
To accomplish this, first a brushfire (wavefront) algorithm and a gvd (Generalized Voronoi Diagram) algorithm are created.
The brushfire algorithm is implemented at C, using the Cffi library to import it to the main python script.
Then, a graph is made in topology.py, topologicalNodes(), with the most important nodes of the gvd, points of the gvd with one neighbor (leafs), points with 3 or more neighbors (junctions) and points with 2 neighbors in which a local minima or maxima of the brushfire value is detected. The last ones are candidate door nodes.
findDoors() method of topology.py eliminates noise candidate doors and keeps only the most possible ones.
findRooms() method of topology.py splits nodes to rooms, using the door nodes as room boundaries.
All data are saved in json files at 'data/' directory.

* **map_to_graph** node reads data from json files, creates the graph of all door nodes and determines the best (minimum distance) sequence to visit all rooms (full graph coverage).
The used graph structure was forked from https://github.com/wylee/Dijkstar. Traveling salesman problem hill climbing solution is influenced from http://www.psychicorigami.com/2007/05/12/tackling-the-travelling-salesman-problem-hill-climbing/.

* **navigation** node reads the planned path of (x,y, theta) targets and executes the navigation using the Navigation Stack (my_navigation package). It computes the starting room in which the robot lies and starts the coverage process from there. For each room, it navigates to all targets and coveres map's obstacles. The use of the Navigation Stack is based on https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/.

* **coverage** node reads constantly robot's pose from the AMCL node and updates the coverage field using Ray Casting and brushfire in the nearby area. The configurations of the sensors are given in the config/config.yaml.

* **coverage_number_subscriber** node reads the coverage field and keeps the number of times each obstacle has been covered.

* **coverage_specs_subscriber** node reads the coverage field and keeps the angles at which each obstacle has been covered. The FOV of the sensors has been splited in half, the positive and the negative bin. An angle metric has been created to check the uniformity of the distribution between the two bins.

##Launchers
* **topology_launcher** : Executes topological analysis (map_to_topology node).
* **graph_launcher** : Executes path planning of map for given sensors (map_to_graph node).
* **navigation_launcher** : Executes autonomous coverage of a map.
* **manual_navigation** : Executes manual navigation in which the user gives each target through rviz.
