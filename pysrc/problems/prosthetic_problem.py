"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""
from pysrc.utilities.tiles import loadTiles, getTiles
import numpy as np
from pysrc.utilities.max_min_finder import *
import numpy
from math import floor

__author__ = 'alex'


class Prosthetic_Experiment(object):
    """ Based on
        A.L. Edwards, M.R. Dawson, J.S. Hebert, R.S. Sutton, K.M. Chan, P.M. Pilarski,
        Adaptive Switching in Practice: Improving Myoelectric Prosthesis Performance through Reinforcement Learning,
        Proc. of MEC14: Myoelectric Controls Symposium, Fredericton, New Brunswick, August 18-22, 2014, pp. 69-73
    """

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']
        self.feature_vector = np.zeros(self.num_tilings)
        self.last_phi = None
        self.last_switch_value = None
        self.num_bins = 6
        self.phi = np.zeros(self.memory_size)
        try:
            self.rl_lambda = config['lmbda']
        except KeyError:
            pass
        try:
            self.normalizer = config['normalizer']
        except KeyError:
            pass

        self.i = 0
        print(config)

    def step(self, obs):
        """Given a set of observations generates the next Phi, Reward, and Gamma for a RL agent
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :returns config: a dictionary with the parameters for the next step in the learning algorithm.
        """
        config = dict()
        config['phi'] = self.last_phi                           # define vals phi
        state = self.get_state(obs)                             # get our current state
        state = self.normalize_state(self.normalizer, state)    # Normalize features within 0-1
        find_invalid(state, obs)                                # Fail if values in state > 1
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

    @staticmethod
    def get_reward(obs):
        """
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :return reward: the cumulant for our learning-algorithm
        """
        # shoulder = obs['vel1']
        # if abs(shoulder) > 0.2:
        #     return 1
        # else:
        #     return 0
        return obs['pos5']

    @staticmethod
    def get_state(obs):
        """
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :return state: a list with the state-values for a time-step
        """
        return [
            obs['pos1'],
            obs['pos2'],
            obs['pos3'],
            obs['pos4'],
            obs['pos5'],
            obs['vel1'],
            obs['vel2'],
            obs['vel3'],
            obs['vel4'],
            obs['vel5'],
            obs['load5']
        ]

    @staticmethod
    def normalize_state(normalizer, state):
        for i in range(len(state)):
            (high, low) = normalizer[i]
            state[i] -= low
            if high - low != 0:
                state[i] = floor((state[i] / (high - low)) * (10**4)) / (10**4)


        return state


class Biorob2012Experiment(Prosthetic_Experiment):

    def __init__(self, config):

        self.starting_element = 0

        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']

        self.feature_vector = np.zeros(self.num_tilings)
        self.last_phi = []
        self.last_switch_value = None
        self.rl_lambda = config['lmbda']
        self.num_bins = 10
        self.alphas = [0.95]
        # self.decay = 0.99
        self.decay = 0.95

        self.velocity_1 = np.zeros(len(self.alphas))
        self.velocity_2 = np.zeros(len(self.alphas))
        self.velocity_4 = np.zeros(len(self.alphas))
        self.velocity_5 = np.zeros(len(self.alphas))

        self.pos_1 = 0
        self.pos_2 = 0
        self.pos_3 = 0
        self.pos_4 = 0
        self.pos_5 = 0
        self.emg_1 = 0
        self.emg_2 = 0
        self.emg_3 = 0

        self.feature_vector_last = 0
        try:
            self.normalizer = config['normalizer']
        except KeyError:
            pass

        try:
            self.obs_keys = config['obs_keys']
        except KeyError:
            pass

    def get_state(self, obs):
        """
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :return state: a list with the state-values for a time-step

        """
        self.pos_1 = self.pos_1 * self.decay + (1 - self.decay) * obs['pos1']
        self.pos_2 = self.pos_2 * self.decay + (1 - self.decay) * obs['pos2']
        self.pos_3 = self.pos_3 * self.decay + (1 - self.decay) * obs['pos3']
        self.pos_4 = self.pos_4 * self.decay + (1 - self.decay) * obs['pos4']
        self.pos_5 = self.pos_5 * self.decay + (1 - self.decay) * obs['pos5']
        self.emg_1 = self.emg_1 * self.decay + (1 - self.decay) * abs(obs['emg1'])
        self.emg_2 = self.emg_2 * self.decay + (1 - self.decay) * abs(obs['emg2'])
        self.emg_3 = self.emg_3 * self.decay + (1 - self.decay) * abs(obs['emg3'])

        state = np.array([
            obs['pos1'],
            obs['pos2'],
            obs['pos3'],
            obs['pos4'],
            obs['pos5'],
            obs['vel1'],
            obs['vel2'],
            obs['vel3'],
            obs['vel4'],
            obs['vel5'],
            # obs['load1'],
            # obs['load2'],
            # obs['load3'],
            # obs['load4'],
            # obs['load5'],
            obs['active_joint'],
            obs['is_moving1'],
            obs['is_moving2'],
            obs['is_moving3'],
            obs['is_moving4'],
            obs['is_moving5'],
            self.emg_1,
            self.emg_2,
            self.emg_3,
            self.pos_1,
            self.pos_2,
            self.pos_3,
            self.pos_4,
            self.pos_5,
        ])
        return state

    def arrange_states(self, state):
        perception = []
        state = np.concatenate((state, [1]))
        shift_factor = self.memory_size / (len(state) - 5)              # the amount of memory we for each tilecoder
        for i in range(len(state) - 5):                                 # for all the other perceptions
            temp = np.concatenate((state[:5], [state[i+5]]))            # add the extra obs to the position obs)
            perception.append(temp)
        return perception

    def get_num_active_features(self, fake_obs):
        """
        :param fake_obs: stand-in for observations so we can determine the length of state
        :returns number of active features: the number of active features we have for this problem
        """
        # the nuber of tilings by number of tiles plus the bias
        return len(self.get_phi(self.get_state(fake_obs))[0]) + 1

    def get_phi(self, state):
        """Multiple tile-coders used. We Compose our position features with every other value in our state.
        :param state: a list with the values returned by get_state()
        :returns feature_vec: a list with the active indices in phi for this time-step
        """
        tile_coders = []
        feature_vec = numpy.array([])                                   # where we store our active features
        shift_factor = self.memory_size / (len(state) - 5)              # the amount of memory we for each tilecoder
        for i in range(len(state) - 5):                                 # for all the other perceptions
            #                        decay position   other     bias
            perception = np.concatenate((state[:5], [state[i+5]]))      # add the extra obs to the position obs)
            tile_coders.append(perception)

            f = np.array(
                    getTiles(
                        numtilings=self.num_tilings,
                        memctable=shift_factor,                         # the amount of memory for each tilecoder
                        floats=perception)
                )
            f += (i * shift_factor)                                     # shift the tiles by the amount we've added on
            f = sorted(f)                                               # we sort to make our verification simpler
            try:
                if f[0] <= feature_vec[len(feature_vec)-1]:
                    print("Tilings are clashing.")                  # notify that our tilings are overlapping
                    raise                                           # fail
            except IndexError:
                pass                                                # the first tiling will have an index error

            feature_vec = np.concatenate((f, feature_vec))          # add our tile-coder to the feature vector
        feature_vec = numpy.concatenate(([1],feature_vec))
        if feature_vec[len(feature_vec) - 1] > self.memory_size:    # if we're using more memory than we have
            print("Exceeding maximum memory")                       # notify

        return feature_vec, tile_coders

    def step(self, obs):
        """
        Given a set of observations generates the next Phi, Reward, and Gamma
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :returns config: a dictionary with the parameters for the next step in the learning algorithm.
        """
        state = self.get_state(obs)                             # get the state
        state = self.normalize_state(self.normalizer, state)    # normalize the state
        find_invalid(state,obs)
        for i in range(len(state)):
            state[i] *= self.num_bins

        phi = np.zeros(self.memory_size)                        # phi is the size
        (self.feature_vector,_) = self.get_phi(state)           # find the new active features
        for i in self.feature_vector:                           # update phi so that...
            phi[i] = 1                                          # all active features are 1
        config = dict()
        config['phi'] = self.last_phi
        config['phinext'] = phi
        self.last_phi = phi
        config['R'] = self.get_reward(obs)
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['l'] = self.rl_lambda
        return config


class TOTDExperiment(Prosthetic_Experiment):

    def __init__(self, config):

        self.starting_element = 0

        self.num_tilings = config['num_tilings']
        self.memory_size = config['memory_size']
        self.gamma = config['gamma']

        self.feature_vector = np.zeros(self.num_tilings)
        self.last_phi = []
        self.last_switch_value = None
        self.rl_lambda = config['lmbda']
        self.num_bins = 10
        self.alphas = [0.95]
        # self.decay = 0.99
        self.decay = 0.95

        self.velocity_1 = np.zeros(len(self.alphas))
        self.velocity_2 = np.zeros(len(self.alphas))
        self.velocity_4 = np.zeros(len(self.alphas))
        self.velocity_5 = np.zeros(len(self.alphas))

        self.pos_1 = 0
        self.pos_2 = 0
        self.pos_3 = 0
        self.pos_4 = 0
        self.pos_5 = 0
        self.emg_1 = 0
        self.emg_2 = 0
        self.emg_3 = 0

        self.feature_vector_last = 0
        try:
            self.normalizer = config['normalizer']
        except KeyError:
            pass

        try:
            self.obs_keys = config['obs_keys']
        except KeyError:
            pass

    def get_state(self, obs):
        """
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :return state: a list with the state-values for a time-step

        """
        self.pos_1 = self.pos_1 * self.decay + (1 - self.decay) * obs['pos1']
        self.pos_2 = self.pos_2 * self.decay + (1 - self.decay) * obs['pos2']
        self.pos_3 = self.pos_3 * self.decay + (1 - self.decay) * obs['pos3']
        self.pos_4 = self.pos_4 * self.decay + (1 - self.decay) * obs['pos4']
        self.pos_5 = self.pos_5 * self.decay + (1 - self.decay) * obs['pos5']
        self.emg_1 = self.emg_1 * self.decay + (1 - self.decay) * abs(obs['emg1'])
        self.emg_2 = self.emg_2 * self.decay + (1 - self.decay) * abs(obs['emg2'])
        self.emg_3 = self.emg_3 * self.decay + (1 - self.decay) * abs(obs['emg3'])

        state = np.array([
            obs['pos5'],
            obs['vel5'],
            obs['emg1'],
            obs['emg2']
            ])
        return state

    def get_num_active_features(self, fake_obs):
        """
        :param fake_obs: stand-in for observations so we can determine the length of state
        :returns number of active features: the number of active features we have for this problem
        """
        # the nuber of tilings by number of tiles plus the bias
        return len(self.get_phi(self.get_state(fake_obs))[0]) + 1

    def get_phi(self, state):
        """Multiple tile-coders used. We Compose our position features with every other value in our state.
        :param state: a list with the values returned by get_state()
        :returns feature_vec: a list with the active indices in phi for this time-step
        """
        f = np.array(
                    getTiles(
                        numtilings=self.num_tilings,
                        memctable=self.memory_size,                         # the amount of memory for each tilecoder
                        floats=state)
        )
        f = numpy.concatenate(([1], f))
        return f, None

    def step(self, obs):
        """
        Given a set of observations generates the next Phi, Reward, and Gamma
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :returns config: a dictionary with the parameters for the next step in the learning algorithm.
        """
        state = self.get_state(obs)                             # get the state
        state = self.normalize_state(self.normalizer, state)    # normalize the state
        find_invalid(state,obs)
        for i in range(len(state)):
            state[i] *= self.num_bins

        phi = np.zeros(self.memory_size)                        # phi is the size
        (self.feature_vector,_) = self.get_phi(state)           # find the new active features
        for i in self.feature_vector:                           # update phi so that...
            phi[i] = 1                                          # all active features are 1
        config = dict()
        config['phi'] = self.last_phi
        config['phinext'] = phi
        self.last_phi = phi
        config['R'] = self.get_reward(obs)
        config['gnext'] = self.gamma
        config['g'] = self.gamma
        config['l'] = self.rl_lambda
        return config

    @staticmethod
    def get_reward(obs):
        """
        :param obs: a dictionary where each key is the title of an observation. This represents the observations for a
        specific time-step.
        :return reward: the cumulant for our learning-algorithm
        """
        return obs['pos5']
