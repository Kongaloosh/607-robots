import numpy as np


class TDPrediction(object):

    def __init__(self, number_of_features):
        self.number_of_features = number_of_features
        self.th = np.zeros(self.number_of_features)

    def estimate(self, state):
        return np.dot(self.th, state)
