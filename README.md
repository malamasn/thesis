# thesis

This repository contains all of the ROS packages developed for my thesis at Aristotle University of Thessaloniki.

## Abstract


Over the last years a rapid growth of the robotics industry has been noticed. Despite the fact that they were primarily used for military applications, nowadays many robotic applications have emerged trying to help people to deal with both everyday and professional tasks.

An important field of robotics applications is the unmanned ground vehicle navigation in known or unknown environments. There is a vast variety of such systems that have already been developed, such as autonomous vehicles, automated house cleaning robots, autonomous real time inventorying, mapping unknown areas etc.

The present Diploma Thesis focuses on studying and solving the problem of the fast and optimal autonomous inventorying of any known 2D warehouse. This problem consists of three sub problems: a) the separation of the known area into subareas, b) the computation of the visiting sequence of these subareas, and c) the computation of the full coverage path in each subarea. The area coverage is accomplished using sensors with a priori unknown characteristics. 

A 2D occupancy grid map representing the environment has been used, in order to face these problems. First, a topological analysis of the map is implemented to locate the area's different rooms, according to which the area is separated. Next, the optimal room sequence is computed. Then, a coverage path for each room is computed through many stages of optimization. The evaluation metrics of the process are the complete area coverage and the execution time of both the computations and the navigation. In addition, in the present Diploma Thesis two different navigation strategies have been developed and compared.

Finally, a series of experiments were carried out at each stage of the implementations in order to thoroughly test each part. Maps with different topologies and sensors with different configurations were used to obtain robust results and test the developed process. As for the experiments, they were solely executed in simulation environments.

## Contents
* **report** : LateX source of thesis report (only in Greek)  

## Dependencies

## Installation
