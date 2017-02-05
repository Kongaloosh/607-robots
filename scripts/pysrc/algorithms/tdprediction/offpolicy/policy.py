import numpy as np

__author__ = 'kongaloosh'


class Policy(object):
    """ classdocs """

    def __init__(self, number_of_features, number_of_actions):
        """    Constructor """
        self.weights = np.zeros((number_of_features, number_of_actions))
        self.epsilon = 0

    def action_state_update(self, action, state):
        self.weights[:, action] += (state != 0)*1

    def probability_of_action_given_state(self, action, state):
        print np.dot(state, self.weights)[action]/np.sum(np.dot(state, self.weights))
        return np.dot(state, self.weights)[action]/np.sum(np.dot(state, self.weights))


class EGreedyPolicy(object):
    """ classdocs """

    def __init__(self, number_of_features, number_of_actions):
        """    Constructor """
        self.weights = np.zeros(number_of_features, number_of_actions)
        self.epsilon = 0

    def probability_of_action_given_state(self, action, state):
        if np.argmax(self.weights, state) == action:
            return 1-self.epsilon
        else:
            return self.epsilon