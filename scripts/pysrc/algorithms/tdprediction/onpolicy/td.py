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
