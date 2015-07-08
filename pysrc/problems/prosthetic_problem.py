"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""

# from pysrc.utilities.verifier import Verifier
from pysrc.utilities.tiles import loadTiles
import numpy as np
from pysrc.utilities.max_min_finder import *

__author__ = 'alex'


class Experiment(object):
    """" Analagous to the mdp,  """

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.phi = np.zeros(self.memory_size)
        self.last_phi = None
        print(config)
        self.rl_lambda = config['lmbda']
        self.last_switch_value = None
        self.num_bins = 20
        try:
            self.normalizer = config['normalizer']
        except:
            pass

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

        hand_velocity = obs['vel1']
        if hand_velocity > 0.2:
            config['R'] = 1
        else:
            config['R'] = 0

        self.last_switch_value = obs['switches']
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['l'] = self.rl_lambda
        config['phinext'] = self.phi
        self.last_phi = self.phi
        return config

    @staticmethod
    def get_reward(obs):
        hand_velocity = obs['vel1']

        if hand_velocity > 0.2:
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


class Experiment_With_Context(Experiment):

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.phi = np.zeros(self.memory_size)
        self.last_phi = None
        print(config)
        self.rl_lambda = config['lmbda']
        self.last_switch_value = None
        self.num_bins = 6

        self.alphas = [.1, .001]  # .01,
        self.decay = 0.99

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
            obs['pos1']/4,
            obs['pos2']/4,
            obs['pos4']/4,
            obs['pos5']/4,
            (obs['vel1']+1.5)/3,
            (obs['vel2']+2)/4,
            (obs['vel4']+2)/4,
            (obs['vel5']+3)/6,
            (obs['load5']+2)/4])

        state = np.concatenate((
            state,
            self.velocity_1,
            self.velocity_2,
            self.velocity_4,
            self.velocity_5))

        return state
