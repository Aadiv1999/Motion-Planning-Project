import six
import sys
sys.modules['sklearn.externals.six'] = six

import mlrose

import numpy as np


def TravellingSalsemanProblem(x, y):

    # Create list of city coordinates
    coords_list = []
    for i in range(len(x)):
        coords_list.append([x[i],y[i]])

    # Initialize fitness function object using coords_list
    fitness_coords = mlrose.TravellingSales(coords = coords_list)

    problem_fit = mlrose.TSPOpt(length = len(x), fitness_fn = fitness_coords,
                                maximize=False)

    best_state, best_fitness = mlrose.genetic_alg(problem_fit, random_state = 2)

    print('The best state found is: ', best_state)

    # print('The fitness at the best state is: ', best_fitness)

    return best_state

