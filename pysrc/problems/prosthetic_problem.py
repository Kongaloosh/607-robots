"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""

# from pysrc.utilities.verifier import Verifier
from pysrc.utilities.tiles import loadTiles
import numpy as np
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
        self.num_bins = 6

    def step(self, obs):
        """
            Given a set of features from a file loader, processes the observations
               into a set of features compatible with the problem we're trying to
               achieve.

            for this example:
                1 = gripper
                2 = rotator
                3 = flexion
                4 = elbow
                5 = shoulder

            for each of these:
                pos = position
                vel = velocity
                temp = temperature
                is_moving = movement

            time

            4 tilings,
            0.01 alpha / 0.1
            0.99 lambda
            0.9 gamma
        """
        config = {}
        config['phi'] = self.last_phi
        state = [obs['pos1']/4, obs['pos2']/4, obs['pos4']/4, obs['pos5']/4,
                 (obs['vel1']+1.5)/3, (obs['vel2']+2)/4, (obs['vel4']+2)/4, (obs['vel5']+3)/6,
                 (obs['load5']+2)/4]

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

        self.velocity_1 = 0
        self.velocity_2 = 0
        self.velocity_4 = 0
        self.velocity_5 = 0

        self.pos_1 = 0
        self.pos_2 = 0
        self.pos_4 = 0
        self.pos_5 = 0

    def step(self, obs):
        decay = 0.9
        config = {}
        config['phi'] = self.last_phi

        self.velocity_1 = self.velocity_1*decay + (1-decay) * obs['vel1']
        self.velocity_2 = self.velocity_2*decay + (1-decay) * obs['vel2']
        self.velocity_4 = self.velocity_4*decay + (1-decay) * obs['vel4']
        self.velocity_5 = self.velocity_5*decay + (1-decay) * obs['vel5']

        self.pos_1 = self.pos_1*decay + (1-decay) * obs['pos1']
        self.pos_2 = self.pos_2*decay + (1-decay) * obs['pos2']
        self.pos_4 = self.pos_4*decay + (1-decay) * obs['pos4']
        self.pos_5 = self.pos_5*decay + (1-decay) * obs['pos5']
        state = [
            obs['pos1']/4,
            obs['pos2']/4,
            obs['pos4']/4,
            obs['pos5']/4,
            (obs['vel1']+1.5)/3,
            (obs['vel2']+2)/4,
            (obs['vel4']+2)/4,
            (obs['vel5']+3)/6,
            (self.pos_1+1.5)/3,
            (self.pos_2+2)/4,
            (self.pos_4+2)/4,
            (self.pos_5+3)/6,
            # (self.velocity_1+1.5)/3,
            # (self.velocity_2+2)/4,
            # (self.velocity_4+2)/4,
            # (self.velocity_5+3)/6,
            (obs['load5']+2)/4]

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