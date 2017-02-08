import numpy as np
from scipy.sparse import csc_matrix as sp
from beginner_tutorials.pysrc.algorithms.tdprediction.tdprediction import TDPrediction


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

    def step(self, reward, phi, phinext, gamma, lmbda, gamma_next):
        delta = reward + gamma_next * np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = gamma * lmbda * self.z + phi
        self.th += self.alpha * delta * phi - self.alpha * gamma * phinext * np.dot(phi, self.w)
        self.w += self.beta * (delta - np.dot(phi, self.w) * phi)


class GTDR(GTD):
    """ classdocs """

    def step(self, reward, phi, phinext, gamma, lmbda, gamma_next):
        delta = reward + gamma_next * np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = gamma * lmbda * self.z * (phi == 0.) + (phi != 0.) * phi
        self.th += self.alpha * delta * phi - self.alpha * gamma * phinext * np.dot(phi, self.w)
        self.w += self.beta * (delta - np.dot(phi, self.w) * phi)
