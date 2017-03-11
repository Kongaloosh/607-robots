import numpy as np
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction

class TOTD(TDPrediction):
  """classdocs"""

  def __init__(self, number_of_features, step_size, active_features=1):
    """ Constructor """
    self.number_of_features = number_of_features
    self.th = np.zeros(self.number_of_features)
    self.z = np.zeros(self.number_of_features)
    self._last_estimate = 0.
    self.alpha = step_size / active_features
        
  def initepisode(self):
    self.z = np.zeros(self.number_of_features)
    
  def step(self, phi, reward, phi_next, gamma, lmbda, gamma_next):
    prednext = np.dot(phi_next, self.th)
    delta = reward + gamma_next * prednext - self.predprev
    self.z = g * lmbda * self.z + self.alpha*phi - self.alpha * gamma_next * lmbda * np.dot(self.z, phi) * phi
    self.th += delta * self.z + self.alpha * (self.predprev - np.dot(phi, self.th)) * phi
    self._last_estimate = prednext

  def last_estimate(self):
        return self._last_estimate