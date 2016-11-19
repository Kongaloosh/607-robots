import random
import math
import operator
import numpy as np

__author__ = 'kongaloosh'

class BaseKanervaCoder:
    def __init__(self, _startingPrototypes, _dimensions):
        self.numPrototypes = _startingPrototypes
        self.dimensions = _dimensions
        self.prototypes = np.array([np.random.rand(self.dimensions) for i in range(self.numPrototypes)])
        self.F = np.array([np.random.rand(self.dimensions) for i in range(self.numPrototypes)])
        self.updatedPrototypes = []

    def get_features(self, data):
        closestPrototypesIndxs = np.zeros(self.numPrototypes)
        diffs = np.zeros(self.numPrototypes)

        for i in range(self.numPrototypes):
            diffs[i] = np.linalg.norm(data - self.prototypes[i])

        diffs = np.argsort(diffs)[:10]
        closestPrototypesIndxs[diffs] = 1

        return closestPrototypesIndxs

    def calculate_f(self, data):

        diffs = np.zeros(self.numPrototypes)
        print(data.shape, self.prototypes[0].shape, "HERHEHERHERHERH ")
        for i in range(self.numPrototypes):
            diffs[i] = np.linalg.norm(data - self.prototypes[i])

        diffs = np.sort(diffs)

        for i in range(self.numPrototypes):
            for j in range(self.dimensions):
                state_diff = np.linalg.norm(data - self.prototypes[i])
                sigmoid = (1 + np.where(state_diff == diffs)[0][0]) / self.numPrototypes
                self.F[i][j] = sigmoid*(1-sigmoid)*data[j]


    def update_prototypes(self, g, alpha, delta, phi, th):
        for i in range(self.numPrototypes):
            for j in range(self.dimensions):
                self.prototypes[i][j] += alpha[i]*delta*(phi[i]*g[i][j] + phi[i]*self.F[i][j])

