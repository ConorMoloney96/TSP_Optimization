# -*- coding: utf-8 -*-
"""
Created on Sun Nov 17 20:59:01 2019

@author: User
"""

import math
import random
import matplotlib.pyplot as plt
import numpy

class SimNearestNeighbour(object):
    #Constructor method 
    #Called when an instance of a class is created 
    #Initalizes values
    def __init__(self, coords, T=-1, stopping_T=-1, alpha=-1, stopping_iter=-1, cur_solution = None):
        self.coords = coords
        self.N = len(coords)
        #More iterations improves performance
        self.stopping_iter = 2500000 if stopping_iter == -1 else stopping_iter
        self.iteration = 1
        self.T = math.sqrt(self.N) if T == -1 else T
        #Higher alpha decreases the rate at which the temperature decreases, improves results
        self.alpha = 0.99 if alpha == -1 else alpha
        self.stopping_temp = 1e-8 if stopping_T == -1 else stopping_T
        self.nodes = [(i+1) for i in range(self.N)]
        self.cur_solution = None
        self.best_solution = None
        self.best_fitness = float("Inf")
        self.fitness_list = []
    
    def calculateBestSolution(self, solution):
        #Calculate fitness score of current solution
        self.cur_fitness = self.fitness(solution)
        if(self.cur_fitness < self.best_fitness):
            self.best_fitness = self.cur_fitness
            self.best_solution = solution
        self.fitness_list.append(self.cur_fitness)
        self.cur_solution = self.best_solution
        print("Best fitness obtained: ", self.best_fitness)
        print("Solution: ", self.best_solution)
    
    def nearestNeighbourAlgorithm(self, start_node = None):
        #If no start node is provided pick one at random
        if(start_node == None):
            #Randomly selects a node as first node
            cur_node = random.choice(self.nodes)
        else:
            cur_node = start_node
        #Current solution i.e. path of nodes visited
        solution = [cur_node]
        free_nodes = self.nodes
        free_nodes.remove(cur_node)
        #Iterate until all nodes visited
        while(len(free_nodes) is not 0):
            #Nearest node to current node
            nearest_node = free_nodes[0]
            #nearest_node = min(free_nodes, key=lambda x: self.distance(cur_node, x))
            #get the nearest node i.e. node with minimum distance
            for node in free_nodes:
                node_distance = self.distance(cur_node, node)
                if(node_distance<self.distance(cur_node, nearest_node)):
                   nearest_node = node
            free_nodes.remove(nearest_node)
            solution.append(nearest_node)
            cur_node = nearest_node
        
        self.calculateBestSolution(solution)
        
    
    #Calculate euclidean distance between 2 nodes      
    def distance(self, node_0, node_1, x1 = None, x2 = None, y1 = None, y2 = None):
        #print("Node: ", node_0)
        coord_0, coord_1 = self.coords[node_0-1], self.coords[node_1-1]
        dist =  math.sqrt((coord_0[1] - coord_1[1]) ** 2 + (coord_0[2] - coord_1[2]) ** 2)
        return dist
        #dist = numpy.linalg.norm(coord_1-coord_0)
        #return dist

    #Calculate fitness function for current solution path
    def fitness(self, solution):
        cur_fit = 0
        for i in range(self.N):
            cur_fit += self.distance(solution[i % self.N], solution[(i + 1) % self.N])
        return cur_fit
    
    def alternativeFitness(self, solution):
        fitness = 0
        for i in range(0, (len(solution)-1)):
            fitness += self.distance(solution[i], solution[i+1])
        return fitness
        
    #Probability of accepting new solution if worse than current
    #Dependent on current temperature and difference between new solution and current
    def accept_prob(self, candidate_fitness, temp = None):
        delta = (-abs(candidate_fitness - self.cur_fitness))
        prob =  math.exp(delta / self.T)
        return prob

    def accept(self, candidate):
        candidate_fitness = self.fitness(candidate)
        #If candidate solution has a lower (i.e. better) fitnesss score than current solution automatically accept it
        if(self.cur_fitness > candidate_fitness):
            self.cur_fitness = candidate_fitness
            self.cur_solution = candidate
            if(candidate_fitness < self.best_fitness):
                self.best_fitness, self.best_solution = candidate_fitness, candidate
        #If candidate solution is not an improvment there is still a chance we will accept it
        else:
            if(random.random() < self.accept_prob(candidate_fitness)):
                self.cur_fitness, self.cur_solution = candidate_fitness, candidate

    
    def annealling_algorithm(self, start_node = None):
    
        start_node = self.cur_solution[0]
    
        while(self.iteration < self.stopping_iter and self.T >= self.stopping_temp):
            #Breaks current solution into a list and randomly re-arranges it
            candidate = list(self.cur_solution)
            #Generates random number in the ranges given
            l = random.randint(2, self.N - 1)
            i = random.randint(0, self.N - l)
            #
            candidate[i+1 : (i + l)] = reversed(candidate[i+1 : (i + l)])
            self.accept(candidate)
            #Reduces temperature with each iteration
            self.T *= self.alpha
            self.iteration += 1
            #Keeps a list of all solutions obtained using annealing
            #self.annealing_solutions.add(candidate)

            self.fitness_list.append(self.cur_fitness)

        #self.calculateBestSolution()
        print("Best fitness obtained through simulated annealing: ", self.best_fitness)
        print("Best solution: ", self.best_solution)
        improvement = 100 * (self.fitness_list[0] - self.best_fitness) / (self.fitness_list[0])
        print(f"Improvement over nearest neighbour heuristic: {improvement : .2f}%")
    

        
        
        