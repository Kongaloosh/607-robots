'''
Created on June 9, 2015

@author: A. Rupam Mahmood

Auto-TD by Thomas Degris

'''

import numpy as np
import pylab as pl
from pysrc.algorithms.tdprediction.tdprediction import TDPrediction

class AutoTD(TDPrediction):
  '''
  classdocs
  '''

  def __init__(self, config):
    '''
    Constructor
    '''
    self.nf         = config['nf']
    self.initalpha  = config['initalpha']
    self.truncate   = config['truncate'] 
    self.th     = np.zeros(self.nf)
    self.z      = np.zeros(self.nf)
    self.alpha  = np.ones(self.nf)*self.initalpha
    self.n      = np.ones(self.nf)
    self.ones   = np.ones(self.nf)
    self.h      = np.zeros(self.nf)
    self.mu     = 0.01
    self.tau    = 10000
    self.maxAlphaDelta = 10 # perhaps it is one
    self.delta  = None
    
  def auto(self, delta, phi, zeta):
    absPhi      = np.abs(phi)
    deltaPhi    = phi * delta
    self.n      += (1 / self.tau) * self.alpha * absPhi * (np.abs(self.h*deltaPhi) - self.n)
    deltaBeta   = np.clip(self.h*delta*phi/self.n, -self.maxAlphaDelta, self.maxAlphaDelta) 
    self.alpha  = np.minimum(self.alpha*np.exp(self.mu*deltaBeta), 1/absPhi)
    if self.truncate and np.dot(self.alpha, zeta)>1:
      self.alpha[zeta != 0] = np.minimum(self.alpha, 1 / np.linalg.norm(zeta, ord=1))[zeta != 0]
    self.th     += self.alpha * deltaPhi
    self.h      = self.h * (self.ones - self.alpha * absPhi) + self.alpha * deltaPhi 
        
  def initepisode(self):
    self.z = np.zeros(self.nf)
    
  def step(self, params):
    phi=params['phi']; R=params['R']; phinext=params['phinext']
    g=params['g']; l=params['l']; gnext=params['gnext']
    self.delta = R + gnext*np.dot(phinext,self.th) - np.dot(phi, self.th)
    self.z = g*l*self.z + phi
    #self.auto( self.delta, self.z, np.abs(self.z)*np.maximum(np.abs(self.z), np.abs(phi-gnext*phinext) ))
    self.auto( self.delta, self.z, np.maximum(np.abs(self.z)*np.abs(phi-gnext*phinext), np.abs(self.z)) )
    
    



      
