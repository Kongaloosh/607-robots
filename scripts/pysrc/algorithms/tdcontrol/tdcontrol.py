import numpy as np


class TDControl(object):

    def __init__(self, number_of_features, number_of_actions):
        self.number_of_features = number_of_features
        self.number_of_actions = number_of_actions
        self.th = np.zeros((self.number_of_actions, self.number_of_features))

    def estimate(self, state):
        return np.dot(self.th, state)
