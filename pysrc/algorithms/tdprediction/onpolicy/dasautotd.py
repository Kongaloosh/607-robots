import numpy as np
from scipy.sparse import csc_matrix as sp
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction


class TD(TDPrediction):
    """ classdocs """

    def __init__(self, config):
        """    Constructor """
        self.mu = 0.01
        self.tau = 1/10000.

        self.nf = config['nf']
        self.alpha = np.ones(self.nf)*self.initalpha
        self.th = np.zeros(self.nf)
        self.v = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.ones = np.ones(self.nf)
        self.z = np.zeros(self.nf)
        try:
          self.initalpha = config['initalpha'] / config['active_features']
        except KeyError:
          self.initalpha = config['initalpha']

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        effective_step_size = g * self.alpha * self.z * phinext
        delta = r + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.v = np.max(
            np.abs(delta*phi*self.h),
            (self.tau * effective_step_size * np.abs(delta * phi * self.h) - self.v)
        )
        self.alpha = self.alpha * np.exp(self.mu * delta * phi * self.h) / self.v
        m = np.max(effective_step_size, 1)
        self.alpha /= m
        self.z = g*l*self.z + phi
        self.th += self.alpha*delta*self.z
        self.h = self.h * (self.ones-effective_step_size) + self.alpha * delta * phi