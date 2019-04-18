#!/usr/bin/env python
import numpy as np
import random



class Routing:

    def route_length(self, distances, route):
        total = 0
        num_nodes = len(route)
        for i in range(num_nodes):
            j = (i+1) % num_nodes
            city_1 = route[i]
            city_2 = route[j]
            total += distances[city_1, city_2]
        return total

    def init_random_route(self, length):
        route = range(length)
        random.shuffle(route)
        return route

    def all_pairs(self, size,shuffle=random.shuffle):
        r1 = range(size)
        r2 = range(size)
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

    def hillclimb(self, distances, epochs):
        '''
        hillclimb until either epochs
        is reached or we are at a local optima
        '''
        length = distances.shape[0]
        best = self.init_random_route(length)
        best_score = self.route_length(distances, best)

        iter = 1

        while iter < epochs:
            # examine moves around our current position
            move_made = False
            for next in self.reversed_sections(best):
                if iter >= epochs:
                    break
                # see if this move is better than the current
                next_score = self.route_length(distances, next)
                iter += 1
                if next_score > best_score:
                    best = next
                    best_score = next_score
                    move_made = True
                    break # depth first search

            if not move_made:
                break # we couldn't find a better move
                         # (must be at a local maximum)

        return best, best_score, iter

    def random_restart_hillclimb(self, distances, epochs):
        '''
        repeatedly hillclimb until max_evaluations is reached
        '''
        best = None
        best_score = 0

        iter = 0
        while iter < epochs:
            iters_left = epochs - iter

            sequence, score, iters_made = self.hillclimb(distances, iters_left)
            iter += iters_made
            if score > best_score or best is None:
                best_score = score
                best = sequence

        return best, best_score, iter
