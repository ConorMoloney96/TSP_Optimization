# -*- coding: utf-8 -*-
"""
Created on Sun Nov 17 19:14:45 2019

@author: User
"""

import matplotlib.pyplot as plt
import random
from nearest_neighbour import SimNearestNeighbour


if __name__ == "__main__":
    coordinates = []
    with open("burmesecities.txt", "r") as file:
        for line in file.readlines():
            line = [float(x) for x in line.strip().split(" ")]
            coordinates.append(line)
            
    nn = SimNearestNeighbour(coordinates)
    nn.nearestNeighbourAlgorithm()
    nn.annealling_algorithm()