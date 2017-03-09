from pysrc.algorithms.tdcontrol.tdcontrol import TDControl
import numpy as np

__author__ = 'kongaloosh'


class ActorCritic(TDControl):
    """ TD REPLACING TRACES """

    def __init__(self, number_of_features, number_of_actions, step_size_critic, step_size_actor, step_size_reward, active_features=1):
        """Constructor"""
        super(ActorCritic, self).__init__(number_of_features, number_of_actions)
        self.z = np.zeros((self.number_of_features,self.number_of_actions))
        self.e_critic = np.zeros((self.number_of_features,self.number_of_actions))
        self.e_actor = np.zeros((self.number_of_features,self.number_of_actions))
        self.th_critic = np.zeros((self.number_of_features,self.number_of_actions))
        self.th_actor = np.zeros((self.number_of_features,self.number_of_actions))
        self.step_size_actor = step_size_actor   / active_features
        self.step_size_critic = step_size_critic / active_features
        self.step_size_reward = step_size_reward / active_features
        self._last_estimate = None
        self.reaward_avg = 0.

    def initialize_episode(self):
        """Initialize the episode by killing the traces"""
        self.e_u = np.zeros(self.e_u.shape)
        self.e_v = np.zeros(self.e_v.shape)

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        critic_delta = self.critic_step(phi, reward, phi_next, gamma, lmda, gamma_next)
        action = self.actor_step(phi, phi_next, gamma, lmda, gamma_next, critic_delta)
        self.action = action
        return action

    def critic_step(self, phi, reward, phi_next):
        """
        :returns the critic's delta
        """
        action_next = self.get_action(phi_next)
        delta = reward - self.average_reward + \
            self.gamma * np.dot(self.th_critic[:, action_next], phi_next) - \
            np.dot(self.th_critic[:, self.action], phi)

        self.avg_reward += self.step_size_reward * delta
        self.e_critic[:, self.action] = self.e_critic[:, self.action] * self.lmbda * self.gnext + phi
        self.th_critic[:, self.action] += self.step_size_critic * delta * self.th_critic[:, self.action]
        return delta

    def actor_step(self, gamma, lmbda, phi, phi_next, critic_delta):
        """Updates the """
        action_next = self.softmax(phi_next)
        self.actor_elegibility[:, self.action] = gamma * lmbda * self.actor_elegibility[:, self.action] + phi
        self.actor_weights[:, self.action] += self.actor_step_size * critic_delta * self.actor_elegibility[:, self.action]
        return action_next

    def softmax(self, phi):
        """for a given action, returns the softmax prob"""
        values = np.dot(self.th_actor, phi)
        return np.argmax(map(lambda v: v/sum(values), values))

    def last_estimate(self):
        return self._last_estimate

    def get_action(self, phi):
        return self.softmax(phi)


class ContinuousActorCritic(TDControl):
    """"""

    def __init__(self, number_of_features, step_size_mean, step_size_deviation, step_size_reward, active_features):
        """"""

        self.step_size_mean = step_size_mean
        self.step_size_deviation = step_size_deviation
        self.ste_size_reward = step_size_reward
        self.active_features = active_features
        self.number_of_features = number_of_features

        self.e_critic    = np.zeros(self.number_of_features)
        self.th_critic   = np.zeros(self.number_of_features)
        self.e_mean      = np.zeros(self.number_of_features)
        self.e_sigma     = np.zeros(self.number_of_features)
        self.th_mean     = np.zeros(self.number_of_features)
        self.th_sigma    = np.zeros(self.number_of_features)

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        critic_delta = self.critic_step(phi, reward, phi_next, gamma, lmda, gamma_next)
        return self.actor_step(phi, phi_next, gamma, lmda, gamma_next, critic_delta)

    def critic_step(self, phi, reward, phi_next):
        """
        :returns the critic's delta
        """
        delta = reward  - self.average_reward + self.gamma * np.dot(self.th_critic, phi_next) - np.dot(self.th_critic, phi)
        self.avg_reward += self.step_size_reward * delta
        self.e_critic   = self.e_critic * self.lmbda * self.gnext  + phi
        self.th_critic  += self.step_size_critic * delta * self.th_critic
        return delta

    def actor_step(self, phi, phi_next, gamma, lmbda, gamma_next, critic_delta):
        """
        :returns action
        """
        mean = np.dot(self.th_mean, phi)         # last step's mean
        sigma = np.dot(self.th_sigma, phi)       # last step's deviation

        gradient_mean = (self.action - mean) * phi                # gradients wrt mean and deviation
        gradient_sigma = ((self.action - mean) - sigma**2) * phi
        # mean update
        self.e_mean = self.e_mean * self.lmbd + gradient_mean
        self.th_mean += self.step_size_mean * self.e_mean * critic_delta
        # deviation update
        self.e_sigma = self.e_sigma * lmbda * gamma_next + gradient_sigma
        self.th_sigma += self.step_size_deviation * self.e_sigma * critic_delta
        return mean,sigma

    def get_action(self, phi):
        mean = np.argmax(np.dot(self.th_mean, phi))
        sigma = np.argmax(np.dot(self.th_sigma, phi))
        np.random.normal(mean, sigma)
