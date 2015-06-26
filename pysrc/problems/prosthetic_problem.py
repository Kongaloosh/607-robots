"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""

# from pysrc.utilities.verifier import Verifier
from pysrc.utilities.tiles import loadTiles

__author__ = 'alex'


class Experiment(object):
    """" Analagous to the mdp,  """

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = 4
        self.memory_size = 2**20
        self.gamma = config['gamma']
        self.feature_vector = [0 for item in range(self.num_tilings)]
        self.last_phi = None

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
        config['phi'] = self.feature_vector


        state = [obs['pos1'], obs['pos2'], obs['pos4'], obs['pos5'],
                 obs['vel1'], obs['vel2'], obs['vel4'], obs['vel5'],
                 obs['load5']]


        loadTiles(self.feature_vector, self.starting_element, self.num_tilings, self.memory_size, state)

        hand_velocity = obs['vel5']
        if hand_velocity > 0.2:
            config['R'] = 1
        else:
            config['R'] = 0

        config['gnext'] = self.gamma
        config['g'] = self.gamma

        config['phi'] = self.feature_vector

        return config