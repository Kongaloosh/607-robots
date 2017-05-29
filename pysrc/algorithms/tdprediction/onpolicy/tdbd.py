import numpy as np
from scipy.sparse import csc_matrix as sp
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
        print(self.alpha)

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
        h_update = np.ones(self.nf) - self.alpha * phi * self.z
        self.h = map(updater, zip(h_update, self.h))
        self.h += self.alpha * delta * self.z



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
        self.beta = np.ones(self.nf) * config['beta']
        print(config['active_features'])
        try:
            val = np.exp(config['beta']) / config['active_features']
            self.beta = np.ones(self.nf) * np.log(val)
        except KeyError:
            self.beta = np.ones(self.nf) * config['beta']
        self.alpha = np.exp(self.beta)
        print(self.alpha)


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
        self.beta += phi * self.meta_step_size * delta
        self.alpha = np.exp(self.beta)
        self.th += self.alpha * delta * self.z
        h_update = np.ones(self.nf) - self.alpha * phi * self.z
        self.h = map(updater, zip(h_update, self.h))
        self.h += self.alpha * delta * self.z
