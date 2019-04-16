ROS package to extract topology features from a given map.

map_to_topology node read an ogm and produces a topological graph, detects door nodes and implements room segmentation.
To accomplish this, first a brushfire (wavefront) algorithm and a gvd (Generalized Voronoi Diagram) algorithm are created.
The brushfire algorithm is implemented at C, using the Cffi library to import it to the main python script.
Then, a graph is made in topology.py, topologicalNodes(), with the most important nodes of the gvd, points of the gvd with one neighbor (leafs), points with 3 or more neighbors (junctions) and points with 2 neighbors in which a local minima or maxima of the brushfire value is detected. The last ones are candidate door nodes.
findDoors() method of topology.py eliminates noise candidate doors and keeps only the most possible ones.
findRooms() method of topology.py splits nodes to rooms, using the door nodes as room boundaries.
All data are saved in json files at 'data/' directory.

map_to_graph node reads data from json files, creates the graph of all door nodes and determines the best (minimum distance) sequence to visit all rooms (full graph coverage).
The used graph structure was forked from https://github.com/wylee/Dijkstar.
