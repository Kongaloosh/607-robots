'''
Created on May 2, 2014

@author: A. Rupam Mahmood
'''

import numpy as np
from scipy.sparse import csc_matrix as sp
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction


class TD(TDPrediction):
    """ classdocs """

    def __init__(self, config):
        """    Constructor """
        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        try:
            self.alpha = config['alpha'] / config['active_features']
        except KeyError:
            self.alpha = config['alpha']

    def initepisode(self):
        self.z = np.zeros(self.nf)

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        delta = r + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g*l*self.z + phi
        self.th += self.alpha*delta*self.z


class TDAlphaBound(TD):

    def __init__(self, config):
        """    Constructor """
        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.z = np.zeros(self.nf)
        self.alpha = config['alpha']

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        delta = r + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g * l * self.z + phi
        self.alpha = min(self.alpha, np.abs(np.dot(self.z, (gnext * phinext - phi)))**(-1))
        self.th += self.alpha * delta * self.z


class TDRMSProp(TD):

    def __init__(self, config):
        super(TDRMSProp, self).__init__(config)
        self.decay = config['decay']
        self.eta = self.alpha
        self.gradient_avg = np.zeros(self.nf)

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        delta = r + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g * l * self.z + phi
        self.gradient_avg = self.decay*self.gradient_avg + (1-self.decay)*phi**2  # removed the neg cause of ^2
        self.alpha = self.eta / (np.sqrt(self.gradient_avg)+10.0**(-8))
        self.th += self.alpha * delta * self.z
