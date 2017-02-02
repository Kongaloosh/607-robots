import numpy as np
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction
from pysrc.utilities.kanerva_coding import BaseKanervaCoder
from pysrc.utilities.Prototype_MetaGradientDescent import MetaGradientDescent


class TDR(TDPrediction):
    """ TD REPLACING TRACES """

    def __init__(self, number_of_features, step_size, active_features=1):
        '''Constructor'''
        self.number_of_features = number_of_features
        self.th = np.zeros(self.number_of_features)
        self.z = np.zeros(self.number_of_features)
        self.step_size = step_size / active_features

    def initialize_episode(self):
        self.z = np.zeros(self.number_of_features)
    
    def step(self, phi, reward, phi_next, gamma, lmda, gamma_next):
        delta = reward + gamma_next * np.dot(phi_next, self.th) - np.dot(phi, self.th)
        self.z = gamma_next * lmda * self.z * (phi == 0.) + (phi != 0.) * phi
        self.th += self.step_size * delta * self.z


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
        self.z = g * l * self.z*(phi==0.) + (phi!=0.)*phi
        self.th += self.alpha*delta*self.z
        self.kanerva.update_prototypes(self.alpha, delta, phi, self.th)

    def estimate(self, phi):
        return np.dot(self.kanerva.get_features(phi), self.th)


class TDR_MGD(TDR):

    def __init__(self, config):
        self.number_of_features = config['nf']
        self.th = np.zeros(self.number_of_features)
        self.v = np.zeros(self.number_of_features)
        self.h = np.zeros(self.number_of_features)
        self.ones = np.ones(self.number_of_features)
        self.z = np.zeros(self.number_of_features)
        try:
          self.initalpha = config['alpha'] / config['nf']
        except KeyError:
          self.initalpha = config['alpha']
        self.step_size = np.ones(self.number_of_features) * self.initalpha
        self.mgd = MetaGradientDescent(_startingPrototypes=config['nf'], _dimensions=4)

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

        delta = r + gnext * np.dot(phinext, self.th) - np.dot(phi, self.th)
        self.z = g * l * self.z * (phi==0.) + (phi != 0.) * phi
        self.th += self.step_size*delta*self.z

        self.mgd.update_prototypes(obs, self.step_size, delta, self.th)

    def estimate(self, phi):
        phi = self.mgd.get_features(phi)
        return np.dot(phi, self.th)
