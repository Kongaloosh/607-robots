import os
import sys
sys.path.insert(0, os.getcwd())

import unittest
from pysrc.utilities.file_loader import FileLoader
from pysrc.problems.prosthetic_problem import \
    Biorob2012Experiment, \
    Prosthetic_Experiment
from pysrc.algorithms.tdprediction.onpolicy.td import TD

from pysrc.experiments.prostheticexp import run_one_config

from pysrc.utilities.file_loader import FileLoader

__author__ = 'alex'

class TestFileLoader(FileLoader):

    def __init__(self):
        self.i = 0

    def step(self):
        if self.has_obs():                                  # while we still have experience
            self.i += 1                                     # move to the next obs
            return [1]               # return the next observation
        else:                                               # otherwise we have nothing left
            return None                                     # so return None.

    def has_obs(self):
        return self.i < 2

class TestProb(Prosthetic_Experiment):
    def __init__(self):
        pass
    
    def get_state(obs):
        return [1]

    def get_reward(obs):
        return 1

    def step(self, obs):
        """Given a set of observations generates the next Phi, Reward, and Gamma for a RL agent
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :returns config: a dictionary with the parameters for the next step in the learning algorithm.
        """
        config = dict()
        config['phi'] = self.last_phi                           # define vals phi
        state = self.get_state(obs)                             # get our current state
        state.append(1)                                         # Bias feature

        for i in range(len(state)):
            state[i] *= self.num_bins

        for i in self.feature_vector:
            self.phi[i] = 0

        self.feature_vector = getTiles(
            self.num_tilings,
            self.memory_size,
            state
        )

        for i in self.feature_vector:
            self.phi[i] = 1

        config['R'] = self.get_reward(obs)
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['phinext'] = self.phi
        try:
            config['l'] = self.rl_lambda   # not every algorithm requires a lambda, so we try
        except KeyError:
            pass

        self.last_phi = self.phi
        return config

class TestProstheticexp(unittest.TestCase):
    """ Test for the prosthetic exp"""

    def setUp(self):
        self.configs = {
            # function approximation:
            'num_tilings': 1,
            'memory_size': 1,
            'nf': 1,
            'starting_element': 0,
            'gamma': 1,
            'lmbda':1,
            'alpha':1
        }

    def test_run_one(self):
        p,s = run_one_config(
            file_loader=TestFileLoader(),
            alg=TD(self.configs),
            # prob=Biorob2012Experiment(self.configs))
            prob=TestProb())
        print(p)

    def pos(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())


if __name__ == '__main__':
    unittest.main()

