from pysrc.algorithms.tdcontrol.tdcontrol import TDControl
import numpy as np

__author__ = 'kongaloosh'


class SARSA(TDControl):
    """"""

    def __init__(self, number_of_features, number_of_actions, step_size, active_features=1):
        """Constructor"""
        super(SARSA, self).__init__(number_of_features, number_of_actions)
        self.z = np.zeros((self.number_of_features,self.number_of_actions))
        self.step_size = step_size / active_features
        self._last_estimate = None
        self.action = None

    def initialize_episode(self):
        self.z = np.zeros(self.z.shape)

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
	print(phi_next.shape, self.th.shape)
        action_next = self.get_action(phi_next)
        if self.action:
            delta = reward + gamma_next * np.dot(phi_next, self.th[action_next]) - np.dot(phi, self.th[self.action])
            self.z = gamma * lmda * self.z * (phi == 0.) + (phi != 0.) * phi
            self.th += self.step_size * delta * self.z
            self._last_estimate = np.dot(phi_next, self.th[self.action])
        self.action = action_next
        return self.action

    def softmax(self, phi):
        """for a given action, returns the softmax prob"""
        values = np.dot(self.th, phi)
        return np.argmax(map(lambda v: v/sum(values), values))

    def last_estimate(self):
        return self._last_estimate

    def get_action(self, phi):
        return self.softmax(phi)

