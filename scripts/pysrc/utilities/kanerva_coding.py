import random
import math
import operator
import numpy as np

__author__ = 'travnik'

class BaseKanervaCoder:
    def __init__(self, _startingPrototypes, _dimensions, _numActiveFeatures):
        self.numPrototypes = _startingPrototypes
        self.dimensions = _dimensions
        self.prototypes = np.array([np.random.rand(self.dimensions) for i in range(self.numPrototypes)])
        self.F = np.array([np.random.rand(self.dimensions) for i in range(self.numPrototypes)])
        self.sorted_prototype_diffs_indexs = np.zeros(self.numPrototypes)
        self.g = np.array([np.random.rand(self.dimensions) for i in range(self.numPrototypes)])

        self.numActiveFeatures = _numActiveFeatures

    def get_features(self, data):
        closestPrototypesIndxs = np.zeros(self.numPrototypes)
        diffs = np.zeros(self.numPrototypes)

        for i in range(self.numPrototypes):
            diffs[i] = np.linalg.norm(data - self.prototypes[i])

        self.sorted_prototype_diffs_indexs = np.argsort(diffs)

        diffs = self.sorted_prototype_diffs_indexs[:self.numActiveFeatures]
        closestPrototypesIndxs[diffs] = 1

        return closestPrototypesIndxs

    def calculate_f(self, data):

        sigmoid = self.sorted_prototype_diffs_indexs / self.numPrototypes
        tempF = np.array([np.random.rand(self.numPrototypes) for i in range(self.dimensions)])
        for i in range(self.dimensions):
            tempF[i] = sigmoid*(1-sigmoid)*data[i] # Setting the whole array here instead of one element at a time
        self.F = tempF.T

    def update_prototypes(self, obs, alpha, delta, phi, th):

        self.calculate_f(obs)
        partA = self.g * np.ones(self.g.shape) - (self.g.T * (alpha * phi)).T
        partB = self.F.T * (1 + alpha * (np.ones(th.shape)*delta - th))

        self.g = partA + partB.T

        partA = alpha*delta*phi
        tempPrototypes = np.array([np.random.rand(self.numPrototypes) for i in range(self.dimensions)])
        tempG = self.g.T

        for i in range(self.dimensions):
            tempPrototypes[i] += partA*tempG[i]
