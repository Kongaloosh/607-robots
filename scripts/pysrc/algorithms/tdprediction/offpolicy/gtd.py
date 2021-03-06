import numpy as np
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction


class GTD(TDPrediction):
    """ classdocs """

    def __init__(self,
                 number_of_features,
                 step_size,
                 target_policy,
                 number_of_active_features):
        """    Constructor """
        self.nf = number_of_features
        self.th = np.zeros(self.nf)
        self.w = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.w = np.zeros(self.nf)
        self.alpha = step_size / number_of_active_features
        self.target_policy = target_policy

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, reward, phi, phinext, gamma, lmbda, gamma_next, action):
        delta = reward + gamma_next * np.dot(phinext, self.th) - np.dot(phi, self.th)
        rho = self.target_policy(action)
        self.z = rho * (phi + gamma * lmbda * self.z)
        self.th += self.alpha * (delta * self.z - gamma * (1 - lmbda) * np.dot(self.z, self.w) * phinext)
        self.w += self.beta * (delta * self.z - np.dot(phi, self.w) * phi)


class GTDR(TDPrediction):
    """ classdocs """

    def __init__(self,
                 number_of_features,
                 step_size,
                 target_policy,
                 number_of_active_features,
                 ):
        """    Constructor """
        self.nf = number_of_features
        self.th = np.zeros(self.nf)
        self.w = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.w = np.zeros(self.nf)
        self.alpha = step_size / number_of_active_features
        self.beta = self.alpha / number_of_active_features
        self.target_policy = target_policy

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, phi, reward, phinext, gamma, lmbda, gamma_next, action):
        delta = reward + gamma_next * np.dot(phinext, self.th) - np.dot(phi, self.th)
        rho = self.target_policy(action)
        self.z = rho * ((phi != 0.) * phi + gamma * lmbda * self.z * (phi == 0.))
        self.th += self.alpha * (delta * self.z - gamma * (1 - lmbda) * np.dot(self.z, self.w) * phinext)
        self.w += self.beta * (delta * self.z - np.dot(phi, self.w) * phi)
