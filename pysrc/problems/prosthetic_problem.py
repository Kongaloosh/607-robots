"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""
from pysrc.utilities.tiles import loadTiles, getTiles
import numpy as np
from pysrc.utilities.max_min_finder import *

__author__ = 'alex'


class Prosthetic_Experiment(object):
    """" Analagous to the mdp,  """

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.phi = np.zeros(self.memory_size)
        self.last_phi = None
        self.last_switch_value = None
        self.num_bins = 20
        try: self.rl_lambda = config['lmbda']       # Lambda-less setups are different
        except: pass
        try: self.normalizer = config['normalizer']
        except: pass

    def step(self, obs):
        config = {}
        config['phi'] = self.last_phi
        state = self.get_state(obs)
        state = self.normalize_state(self.normalizer, state)
        find_invalid(state, obs)

        for i in range(len(state)):
            state[i] *= self.num_bins

        for i in self.feature_vector:
            self.phi[i] = 0

        loadTiles(self.feature_vector, self.starting_element, self.num_tilings, self.memory_size, state)

        for i in self.feature_vector:
            self.phi[i] = 1

        config['R'] = self.get_reward(obs)

        self.last_switch_value = obs['switches']
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        try: config['l'] = self.rl_lambda   # not every algorithm requires a lambda, so we try
        except: pass
        config['phinext'] = self.phi
        self.last_phi = self.phi
        return config

    @staticmethod
    def get_reward(obs):
        elbow_velocity = obs['vel3']

        if elbow_velocity > 0.2:
            return 1
        else:
            return 0

    @staticmethod
    def get_state(obs):
        return [obs['pos1']/4, obs['pos2']/4, obs['pos4']/4, obs['pos5']/4,
                (obs['vel1']+1.5)/3, (obs['vel2']+2)/4, (obs['vel4']+2)/4, (obs['vel5']+3)/6,
                (obs['load5']+2)/4]

    @staticmethod
    def normalize_state(normalizer, state):
        for i in range(len(state)):
            (high, low) = normalizer[i]
            state[i] -= low
            state[i] /= (high-low)
        return state


class Prosthetic_Experiment_With_Context(Prosthetic_Experiment):

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.phi = np.zeros(self.memory_size)
        self.last_phi = None
        self.rl_lambda = config['lmbda']
        self.last_switch_value = None
        self.num_bins = 6

        self.alphas = [0.95]  #
        self.decay = 0.95

        self.velocity_1 = np.zeros(len(self.alphas))
        self.velocity_2 = np.zeros(len(self.alphas))
        self.velocity_4 = np.zeros(len(self.alphas))
        self.velocity_5 = np.zeros(len(self.alphas))

        self.pos_1 = 0
        self.pos_2 = 0
        self.pos_4 = 0
        self.pos_5 = 0

        try:
            self.normalizer = config['normalizer']
        except:
            pass

    def get_state(self, obs):
        self.velocity_1 = [
            (self.velocity_1[i]*self.alphas[i] + ((1-self.alphas[i])*obs['vel1']))
            for i in range(len(self.velocity_1))]

        self.velocity_2 = [
            (self.velocity_2[i]*self.alphas[i] + ((1-self.alphas[i])*obs['vel2']))
            for i in range(len(self.velocity_2))]

        self.velocity_4 = [
            (self.velocity_4[i]*self.alphas[i] + ((1-self.alphas[i])*obs['vel4']))
            for i in range(len(self.velocity_4))]

        self.velocity_5 = [
            (self.velocity_5[i]*self.alphas[i] +((1-self.alphas[i])*obs['vel5']))
            for i in range(len(self.velocity_5))]

        self.pos_1 = self.pos_1*self.decay + (1-self.decay) * obs['pos1']
        self.pos_2 = self.pos_2*self.decay + (1-self.decay) * obs['pos2']
        self.pos_4 = self.pos_4*self.decay + (1-self.decay) * obs['pos4']
        self.pos_5 = self.pos_5*self.decay + (1-self.decay) * obs['pos5']

        state = np.array([
            obs['pos1'],
            obs['pos2'],
            obs['pos4'],
            obs['pos5'],
            obs['vel1'],
            obs['vel2'],
            obs['vel4'],
            obs['vel5'],
            obs['load5'],
        ])

        state = np.concatenate((
            state,
            self.velocity_1,
            self.velocity_2,
            self.velocity_4,
            self.velocity_5,
            [self.pos_1, self.pos_2, self.pos_4, self.pos_5]
        ))

        return state

    def step(self, obs):
        config = {}
        config['phi'] = self.last_phi

        state = self.get_state(obs)
        state = self.normalize_state(self.normalizer, state)
        find_invalid(state, obs)

        for i in range(len(state)):
            state[i] *= self.num_bins

        for i in self.feature_vector:
            self.phi[i] = 0

        '''
            Multiple tile-coders used. We load the first half of the states in during the first load
        '''

        loadTiles(self.feature_vector, 0, self.num_tilings, self.memory_size, state[:len(state)/2], [0])
        for i in self.feature_vector:
            self.phi[i] = 1

        loadTiles(self.feature_vector, 0, self.num_tilings, self.memory_size, state[len(state)/2:], [1])
        for i in self.feature_vector:
            self.phi[i] = 1

        config['R'] = self.get_reward(obs)

        self.last_switch_value = obs['switches']
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['l'] = self.rl_lambda
        config['phinext'] = self.phi
        self.last_phi = self.phi
        return config


class Biorob2012Experiment(Prosthetic_Experiment):

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.phi = np.zeros(self.memory_size)
        self.last_phi = None
        self.last_switch_value = None
        self.rl_lambda = config['lmbda']
        self.num_bins = 6
        self.alphas = [0.95]
        self.decay = 0.95
        self.velocity_1 = np.zeros(len(self.alphas))
        self.velocity_2 = np.zeros(len(self.alphas))
        self.velocity_4 = np.zeros(len(self.alphas))
        self.velocity_5 = np.zeros(len(self.alphas))
        self.pos_1 = 0
        self.pos_2 = 0
        self.pos_4 = 0
        self.pos_5 = 0

        try:
            self.normalizer = config['normalizer']
        except:
            pass

    def get_state(self, obs):
        self.pos_1 = self.pos_1*self.decay + (1-self.decay) * obs['pos1']
        self.pos_2 = self.pos_2*self.decay + (1-self.decay) * obs['pos2']
        self.pos_4 = self.pos_4*self.decay + (1-self.decay) * obs['pos4']
        self.pos_5 = self.pos_5*self.decay + (1-self.decay) * obs['pos5']

        state = np.array([
            obs['pos1'],
            obs['pos2'],
            obs['pos4'],
            obs['pos5'],
            obs['vel1'],
            obs['vel2'],
            obs['vel4'],
            obs['vel5'],
            obs['load1'],
            obs['load2'],
            obs['load4'],
            obs['load5'],
            self.pos_1,
            self.pos_2,
            self.pos_4,
            self.pos_5,
        ])

        return state

    def step(self, obs):
        config = {}
        config['phi'] = self.last_phi

        state = self.get_state(obs)
        state = self.normalize_state(self.normalizer, state)
        find_invalid(state, obs)

        for i in range(len(state)):
            state[i] *= self.num_bins

        for i in self.feature_vector:
            self.phi[i] = 0

        '''
            Multiple tile-coders used. We load the first half of the states in during the first load
        '''
        shift_factor = self.memory_size / (len(state) - 4)
        for i in range(len(state) - 4):
            perception = np.concatenate((state[:4], [state[i]]))
            f = np.array(getTiles(
                numtilings=self.num_tilings,
                memctable=self.memory_size,
                floats=perception)
            ) + i * shift_factor

                # (i * )
            # np.concatenate((self.feature_vector, f))

        for i in self.feature_vector:
            self.phi[i] = 1

        config['R'] = self.get_reward(obs)

        self.last_switch_value = obs['switches']
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['l'] = self.rl_lambda
        config['phinext'] = self.phi
        self.last_phi = self.phi
        return config

