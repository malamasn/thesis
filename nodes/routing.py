#!/usr/bin/env python
import numpy as np
import random, math



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

    def reversed_sections(self, route, fixed_edges = False):
        '''generator to return all possible variations
          where the section between two cities are swapped'''
        for i,j in self.all_pairs(len(route)):
            # Skip iterations that change route edges if fixed_edges
            if fixed_edges:
                if not i or not j or i == len(route)-1 or j == len(route)-1:
                    continue
            if i != j:
                copy=route[:]
                if i < j:
                    copy[i:j+1]=reversed(route[i:j+1])
                else:
                    copy[i+1:]=reversed(route[:j])
                    copy[:j]=reversed(route[i+1:])
                if copy != route: # no point returning the same tour
                    yield copy

    def P(self, prev_score, next_score, temperature):
        if next_score > prev_score:
            return 1.0
        else:
            return math.exp(-abs(next_score - prev_score)/temperature)

    def kirkpatrick_cooling(self, start_temp, alpha):
        T = start_temp
        while True:
            yield T
            T = alpha * T


    def hillclimb(self, distances, epochs, fixed_edges = False):
        '''
        hillclimb until either epochs
        is reached or we are at a local optima
        '''
        length = distances.shape[0]
        # best = self.init_random_route(length)
        best = range(length)
        best_score = self.route_length(distances, best)

        iter = 1

        while iter < epochs:
            # examine moves around our current position
            move_made = False
            for next in self.reversed_sections(best, fixed_edges):
                if iter >= epochs:
                    break
                # see if this move is better than the current
                next_score = self.route_length(distances, next)
                iter += 1
                if next_score < best_score:
                    best = next
                    best_score = next_score
                    move_made = True
                    break # depth first search

            if not move_made:
                break # we couldn't find a better move
                         # (must be at a local maximum)

        return best, best_score, iter

    def random_restart_hillclimb(self, distances, epochs, fixed_edges = False):
        '''
        repeatedly hillclimb until epochs is reached
        '''
        best = None
        best_score = 0

        iter = 0
        while iter < epochs:
            iters_left = epochs - iter

            sequence, score, iters_made = self.hillclimb(distances, iters_left, fixed_edges)
            iter += iters_made
            if score < best_score or best is None:
                best_score = score
                best = sequence

        return best, best_score, iter



    def anneal(self, distances, epochs, start_temp, alpha):

        length = distances.shape[0]
        best = range(length)
        best_score = self.route_length(distances, best)
        iter = 0

        cooling_schedule = self.kirkpatrick_cooling(start_temp, alpha)

        for temperature in cooling_schedule:
            done = False
            # examine moves around our current position
            for next in self.reversed_sections(best):
                if iter >= epochs:
                    done = True
                    break

                next_score = self.route_length(distances, best)
                iter += 1

                # probablistically accept this solution
                # always accepting better solutions
                p = self.P(best_score, next_score, temperature)
                if random.random() < p:
                    best = next
                    best_score = next_score
                    break
            # see if completely finished
            if done:
                 break

        return best, best_score, iter


    def step_hillclimb(self, distances, epochs, step):
        '''
        keep distances equal to step and minimize all others
        until the sequence is found or epochs is reached
        '''
        length = distances.shape[0]
        best = range(length)
        best_score = self.route_length(distances, best)
        iter = 0

        for i in range(length-2):
            if iter >= epochs:
                break
            min = distances[best[i]][best[i+1]]
            if min <= step:
                continue
            else:
                index = i+1
                for j in range(i+2,length):
                    dist = distances[best[i]][best[j]]
                    if dist < min:
                        min = dist
                        index = j
                        iter += 1
                    if min <= step:
                        break
                best[i+1], best[index] = best[index], best[i+1]
                best_score = self.route_length(distances, best)

        return best, best_score, iter
