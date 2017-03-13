from pysrc.algorithms.tdcontrol.tdcontrol import TDControl
import numpy as np

__author__ = 'kongaloosh'


class ActorCritic(TDControl):
    """ TD REPLACING TRACES """

    def __init__(self, number_of_features, number_of_actions, step_size_critic, step_size_actor, step_size_reward,
                 active_features=1):
        """Constructor"""
        super(ActorCritic, self).__init__(number_of_features, number_of_actions)
        self.action = None
        self.z = np.zeros((self.number_of_actions, self.number_of_features))
        self.e_critic = np.zeros((self.number_of_features))
        self.e_actor = np.zeros((self.number_of_features, self.number_of_actions))
        self.th_critic = np.zeros((self.number_of_features))
        self.th_actor = np.zeros((self.number_of_features, self.number_of_actions))
        self.step_size_actor = step_size_actor / active_features
        self.step_size_critic = step_size_critic / active_features
        self.step_size_reward = step_size_reward / active_features
        self._last_estimate = None
        self.average_reward = 0.

    def initialize_episode(self):
        """Initialize the episode by killing the traces"""
        self.e_u = np.zeros(self.e_u.shape)
        self.e_v = np.zeros(self.e_v.shape)

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        critic_delta = self.critic_step(phi, reward, phi_next, gamma, lmda, gamma_next)
        self.action = self.actor_step(phi, phi_next, gamma, lmda, critic_delta)
        return self.action

    def critic_step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        """
        :returns the critic's delta
        """
        delta = reward - self.average_reward + \
                gamma * np.dot(self.th_critic, phi_next) - \
                np.dot(self.th_critic, phi)

        self.average_reward += self.step_size_reward * delta
        self.e_critic = self.e_critic * lmda * gamma_next + phi
        self.th_critic += self.step_size_critic * delta * self.th_critic
        return delta

    def actor_step(self, phi, phi_next, gamma, lmbda, critic_delta):
        """Updates the """
        action_next = self.get_action(phi_next)
        print(self.e_actor.shape, self.e_actor[:, self.action].shape, self.th_actor[:,self.action], self.action, action_next)
        self.e_actor[:, self.action] = gamma * lmbda * self.e_actor[:, self.action] + phi
        self.th_actor[:, self.action] += self.step_size_actor * critic_delta * self.e_actor[:, self.action]
        return action_next

    def softmax(self, phi):
        """for a given action, returns the softmax prob"""
        if self.action:
            values = np.exp(np.dot(phi, self.th_actor))
            softmax = map(lambda v: v / sum(values), values)
            action = np.random.choice(len(softmax), 1, p=softmax)[0]
            print("action", action)
            return action
        else:
            self.action = np.random.choice(len(np.dot(phi, self.th_actor)),1)[0]
            return self.action

    def last_estimate(self):
        return self._last_estimate

    def get_action(self, phi):
        action = self.softmax(phi)
        print action
        return action


class ContinuousActorCritic(TDControl):
    """"""

    def __init__(self, number_of_features, step_size_mean, step_size_deviation, step_size_critic, step_size_reward,
                 active_features):
        """"""

        self.step_size_mean = step_size_mean
        self.step_size_deviation = step_size_deviation
        self.step_size_critic = step_size_critic
        self.step_size_reward = step_size_reward
        self.active_features = active_features
        self.number_of_features = number_of_features

        self.e_critic = np.zeros(self.number_of_features)
        self.th_critic = np.zeros(self.number_of_features)
        self.e_mean = np.zeros(self.number_of_features)
        self.e_sigma = np.zeros(self.number_of_features)
        self.th_mean = np.zeros(self.number_of_features)
        self.th_sigma = np.ones(self.number_of_features)

        self.average_reward = 0.

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        critic_delta = self.critic_step(phi, reward, phi_next, gamma, lmda, gamma_next)
        print("critic", critic_delta)
        return self.actor_step(phi, phi_next, gamma, lmda, gamma_next, critic_delta)

    def critic_step(self, phi, reward, phi_next, gamma, lmda, gnext):
        """
        :returns the critic's delta
        """
        delta = reward - self.average_reward + gamma * np.dot(self.th_critic, phi_next) - np.dot(self.th_critic, phi)
        self.average_reward += self.step_size_reward * delta
        self.e_critic = self.e_critic * lmda * gnext + phi
        self.th_critic += self.step_size_critic * delta * self.e_critic
        return delta

    def actor_step(self, phi, phi_next, gamma, lmbda, gamma_next, critic_delta):
        """
        :returns action
        """
        mean = np.dot(self.th_mean, phi)  # last step's mean
        sigma = np.dot(self.th_sigma, phi)  # last step's deviation
        gradient_mean = (self.action - mean) * phi  # gradients wrt mean and deviation
        gradient_sigma = ((self.action - mean) - sigma ** 2) * phi
        # mean update
        self.e_mean = self.e_mean * lmbda + gradient_mean
        self.th_mean += self.step_size_mean * self.e_mean * critic_delta
        # deviation update
        self.e_sigma = self.e_sigma * lmbda * gamma_next + gradient_sigma
        self.th_sigma += self.step_size_deviation * self.e_sigma * critic_delta
        return mean, sigma

    def get_action(self, phi):
        mean = np.dot(self.th_mean, phi)
        sigma = np.exp(np.dot(self.th_sigma, phi))
        if sigma == 0:
            sigma = 0.2
        print(mean, sigma)
        self.action = np.random.normal(mean, sigma)
        return self.action
