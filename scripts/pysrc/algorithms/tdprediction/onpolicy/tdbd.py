import numpy as np
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction

__author__ = 'kongaloosh'


def updater(vals):
    h_update, h = vals
    if h_update > 0:
        return h * h_update
    else:
        return 0


class TDBD(TDPrediction):
    """ classdocs """

    def __init__(self, config):
        '''Constructor'''
        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.beta = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.meta_step_size = config['meta_step_size']                # todo: pull from config
        try:
            val = np.exp(config['beta']) / config['active_features']
            self.beta = np.ones(self.nf) * np.log(val)
        except KeyError:
            self.beta = np.ones(self.nf) * config['beta']

        self.alpha = np.exp(self.beta)

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
        self.z = g * l * self.z + phi
        self.beta += phi * self.meta_step_size * delta
        self.alpha = np.exp(self.beta)
        self.th += self.alpha * delta * self.z
        h_update = [1 - self.alpha * phi**2]
        if h_update > 0:
            self.h = self.h * h_update + self.alpha * delta * phi
        else:
            self.h = self.h * 0 + self.alpha * delta * phi


class TDBDR(TDPrediction):
    """TD-based Incremental Delta Bar Delta"""

    def __init__(self, number_of_features, step_size, meta_step_size, active_features=1):
        '''Constructor'''
        self.nf = number_of_features
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.beta = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.meta_step_size = meta_step_size               # todo: pull from config
        self.beta = np.ones(self.nf) * step_size
        self.alpha = np.exp(self.beta)

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        delta = reward + gamma_next * np.dot(phi_next, self.th) - np.dot(phi, self.th)
        self.z = g * lmda * self.z * (phi == 0.) + (phi != 0.) * phi
        self.beta += phi * self.meta_step_size * delta
        self.alpha = np.exp(self.beta)
        self.th += self.alpha * delta * self.z
        h_update = np.ones(self.nf) - self.alpha * phi**2
        self.h = map(updater, zip(h_update, self.h))
        self.h += self.alpha * delta * phi