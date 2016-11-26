import numpy as np
from scipy.sparse import csc_matrix as sp
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction
from pysrc.utilities.kanerva_coding import BaseKanervaCoder
from pysrc.utilities.Prototype_MetaGradientDescent import MetaGradientDescent

class TDR(TDPrediction):
    """ TD REPLACING TRACES """

    def __init__(self, config):
        '''Constructor'''
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
        phi=params['phi']; R=params['R']; phinext=params['phinext']
        g=params['g']; l=params['l']; gnext=params['gnext']
        delta = R + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g*l*self.z*(phi==0.) + (phi!=0.)*phi
        self.th += self.alpha*delta*self.z

    def quick_step(self, params):
        """ STEP WHICH LEVERAGES SPARSITY """
        phi = params['phi']
        R = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        delta = R + gnext*sp.dot(sp(phinext), self.th) - sp.dot(sp(phi), self.th)
        self.z = g*l*self.z*(phi == 0.) + (phi != 0.)*phi
        self.th += self.alpha*delta*self.z

class TDR_Kanerva(TDPrediction):

    def __init__(self, config):
        self.mu = 0.01
        self.tau = 1 / 10000.

        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.v = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.ones = np.ones(self.nf)
        self.z = np.zeros(self.nf)

        try:
          self.initalpha = config['alpha'] / config['active_features']
        except KeyError:
          self.initalpha = config['alpha']
        self.alpha = np.ones(self.nf) * self.initalpha
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=1024,
            _dimensions=4,
            _numActiveFeatures=config['active_features'])

    def step(self, params):
        phi = params['phi']
        R = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']
        self.kanerva.calculate_f(phi)

        phi = self.kanerva.get_features(phi)
        phinext = self.kanerva.get_features(phinext)

        delta = R + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g*l*self.z*(phi==0.) + (phi!=0.)*phi
        self.th += self.alpha*delta*self.z
        self.kanerva.update_prototypes(self.alpha, delta, phi, self.th)

    def estimate(self, phi):
        return np.dot(self.kanerva.get_features(phi), self.th)


class TDR_MGD(TDR):

    def __init__(self, config):

        self.nf = config['nf']
        self.th = np.zeros(self.nf)
        self.v = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.ones = np.ones(self.nf)
        self.z = np.zeros(self.nf)

        try:
          self.initalpha = config['alpha'] / config['active_features']
        except KeyError:
          self.initalpha = config['alpha']
        self.alpha = np.ones(self.nf) * self.initalpha
        self.mgd = MetaGradientDescent(_startingPrototypes=1024, _dimensions=4)

    def step(self, params):
        phi = params['phi']
        r = params['R']
        phinext = params['phinext']
        g = params['g']
        l = params['l']
        gnext = params['gnext']

        obs = phi

        phi = self.mgd.get_features(phi)
        phinext = self.mgd.get_features(phinext)

        delta = r + gnext*np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g*l*self.z*(phi==0.) + (phi!=0.)*phi
        self.th += self.alpha*delta*self.z

        self.mgd.update_prototypes(obs, self.alpha, delta, self.th)


    def estimate(self, phi):
        phi = self.mgd.get_features(phi)
        return np.dot(phi, self.th)
