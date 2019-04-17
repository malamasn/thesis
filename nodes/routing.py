#!/usr/bin/env python
import numpy as np



class Routing:

    def route_length(self, distances, tour):
        total = 0
        num_nodes = len(tour)
        for i in range(num_nodes):
            j = (i+1) % num_nodes
            city_1 = tour[i]
            city_2 = tour[j]
            total += distances[city_1, city_2]
        return total
