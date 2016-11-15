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
        self.th = np.zeros(self.nf)
        self.v = np.zeros(self.nf)
        self.h = np.zeros(self.nf)
        self.ones = np.ones(self.nf)
        self.z = np.zeros(self.nf)
        try:
          self.initalpha = config['initalpha'] / config['active_features']
        except KeyError:
          self.initalpha = config['initalpha']
        print("here", self.initalpha)
        self.alpha = np.ones(self.nf)*self.initalpha
        print("here", self.alpha)

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
        self.v = np.maximum(
            np.abs(delta*phi*self.h),
            (self.tau * effective_step_size * np.abs(delta * phi * self.h) - self.v)
        )

        self.alpha = self.alpha * np.exp(np.where(self.v == 0, 0, (self.mu * delta * phi * self.h) / self.v))

        # safe = self.v != 0
        # alpha_update = (self.mu * delta * phi * self.h)
        #
        # print(
        #     """"
        #         safe            {0}
        #         alpha update    {1}
        #         placing         {2}
        #     """.format(
        #         safe,
        #         alpha_update,
        #         np.place(np.zeros_like(alpha_update), safe,  alpha_update[safe]/ self.v[safe])
        #     )
        # )
        #
        # self.alpha = self.alpha * np.exp(np.place(np.zeros_like(alpha_update), safe,  alpha_update[safe]/ self.v[safe]))
        m = np.maximum(effective_step_size, self.ones)
        self.alpha /= m
        self.z = g*l*self.z + phi
        self.th += self.alpha*delta*self.z
        self.h = self.h * (self.ones-effective_step_size) + self.alpha * delta * phi
        