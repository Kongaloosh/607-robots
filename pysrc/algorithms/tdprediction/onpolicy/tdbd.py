import numpy as np
from scipy.sparse import csc_matrix as sp
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction

__author__ = 'kongaloosh'


class TDBD(TDPrediction):
    """ classdocs """

    def __init__(self, config):
        """    Constructor """
        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.beta = np.zeros(self.nf)

        try:
            self.alpha = np.ones(self.nf) * config['alpha'] / config['active_features']
        except KeyError:
            self.alpha = np.ones(self.nf) * config['alpha']

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        delta = r + gnext * np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g * l * self.z + phi
        self.th += self.alpha * delta * self.z


class TDBDR(TDPrediction):
    """TD-based Incremental Delta Bar Delta"""

    def __init__(self, config):
        '''Constructor'''
        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.beta = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.meta_step_size = config['meta_step_size']                # todo: pull from config
        try:
            self.alpha = np.ones(self.nf) * config['alpha'] / config['active_features']
        except KeyError:
            self.alpha = np.ones(self.nf) * config['alpha']

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, params):
        phi = params['phi']
        R = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']
        delta = R + gnext * np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g * l * self.z * (phi == 0.) + (phi != 0.) * phi
        self.beta += self.meta_step_size + delta + phi
        self.alpha = np.exp(self.beta)
        self.th += self.alpha * delta * self.z
        self.h = self.h * [1 - self.alpha * phi**2] + self.alpha * delta * phi
