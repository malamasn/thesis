#!/usr/bin/env python
import numpy as np
import random



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

    def init_random_route(self, length):
        route = range(length)
        random.shuffle(route)
        return route

    def all_pairs(self, size,shuffle=random.shuffle):
        r1=range(size)
        r2=range(size)
        if shuffle:
            shuffle(r1)
            shuffle(r2)
        for i in r1:
            for j in r2:
                yield (i,j)

    def swapped_nodes(self, route):
        '''generator to create all possible variations
          where two cities have been swapped'''
        for i,j in self.all_pairs(len(route)):
            if i < j:
                copy=route[:]
                copy[i],copy[j]=route[j],route[i]
                yield copy

    def reversed_sections(self, route):
        '''generator to return all possible variations
          where the section between two cities are swapped'''
        for i,j in self.all_pairs(len(route)):
            if i != j:
                copy=route[:]
                if i < j:
                    copy[i:j+1]=reversed(route[i:j+1])
                else:
                    copy[i+1:]=reversed(route[:j])
                    copy[:j]=reversed(route[i+1:])
                if copy != route: # no point returning the same tour
                    yield copy
