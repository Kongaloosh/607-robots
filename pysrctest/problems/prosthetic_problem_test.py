import os
import sys
sys.path.insert(0, os.getcwd())
import unittest
from pysrc.problems.prosthetic_problem import Prosthetic_Experiment, Biorob2012Experiment


class BasicProblem (unittest.TestCase):
    """defines common setup"""

    def setUp(self):
        configs = {
            'num_tilings':5,
            'memory_size':2**10,
            'gamma':0,
            'lmbda':0,
            'normalizer':[(100,0) for i in range(23)],
        }

        self.biorob = Biorob2012Experiment(configs)
        self.prosthetic_experiment = Prosthetic_Experiment(configs)


class TestProstheticProblem (BasicProblem):
    """Ensures that the state and the rewards are correctly calculated by problem"""
    #TODO: abstract out get-phi and make sure everything's fine

    def setUp(self):
        print('BasicTest.setUp')
        BasicProblem.setUp(self)

    # def test_get_reward_prosthetic_experiment(self):
    #     """Should return 1 if abs value of elbow is > 0.2 otherwise, 0"""
    #     self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':1}),1)
    #     self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':-1}),1)
    #
    #     self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':0.1}),0)
    #     self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':-0.1}),0)

    # def test_get_reward_biorob(self):
    #     """Should return 1 if abs value of elbow is > 0.2 otherwise, 0"""
    #     self.assertEquals(self.biorob.get_reward({'vel4':1}),1)
    #     self.assertEquals(self.biorob.get_reward({'vel4':-1}),1)
    #
    #     self.assertEquals(self.biorob.get_reward({'vel4':0.1}),0)
    #     self.assertEquals(self.biorob.get_reward({'vel4':-0.1}),0)

    # def test_get_state_prosthetic_experiment(self):
    #     """Should only return the states which are not labeled random_val_#"""
    #     obs = {
    #         'random_val_1': 100,
    #         'pos1': 1,
    #         'random_val_2': 99,
    #         'pos2': 2,
    #         'random_val_3': 98,
    #         'pos3': 3,
    #         'random_val_4': 97,
    #         'pos5': 4,
    #         'random_val_5': 96,
    #         'vel1': 5,
    #         'random_val_6': 95,
    #         'vel2': 6,
    #         'random_val_7': 94,
    #         'vel3': 7,
    #         'random_val_8': 93,
    #         'vel5': 8,
    #         'random_val_9': 92,
    #         'load5': 9,
    #         'random_val_9': 91,
    #     }
    #     state_true = [1,2,3,4,5,6,7,8,9]
    #     state = self.prosthetic_experiment.get_state(obs)
    #     self.assertItemsEqual(state, state_true)

    def test_get_state_biorob(self):
        """Should only return the states which are not labeled random_val_# AND the decay traces"""
        obs = {
            'pos1': 1,
            'pos2': 2,
            'pos3': 3,
            'pos4': 4,
            'pos5': 5,
            'vel1': 6,
            'vel2': 7,
            'vel3': 8,
            'vel4': 9,
            'vel5': 10,
            'load1': 11,
            'load2': 12,
            'load3': 13,
            'load4': 14,
            'load5': 15,
            'emg1': 16,
            'emg2': 17,
            'emg3': 18,
        }

        state_true = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 1),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 2),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 3),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 4),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 5),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 16),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 17),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 18)]

        state = self.biorob.get_state(obs)

        # check to make sure states are being properly formatted
        self.assertEqual(len(state), len(state_true))
        self.assertItemsEqual(state, state_true)

        state = self.biorob.step(obs)
        phi = state['phinext']

        obs = {
            'pos1': 18,
            'pos2': 17,
            'pos3': 16,
            'pos4': 15,
            'pos5': 14,
            'vel1': 13,
            'vel2': 12,
            'vel3': 11,
            'vel4': 10,
            'vel5': 9,
            'load1': 8,
            'load2': 7,
            'load3': 6,
            'load4': 5,
            'load5': 4,
            'emg1': 3,
            'emg2': 2,
            'emg3': 1,
        }

        state = self.biorob.step(obs)

        self.assertItemsEqual(phi, state['phi'])    # check to make sure phis are updated correctly
        self.assertEqual(obs['pos1'], state['R'])   # check rewards formation
        self.assertEqual(state['g'], state['gnext'])
        self.assertEqual(state['g'], 0)
        self.assertEqual(state['l'], 0)

        # true_arrangement = [[1,2,3,4,5,6],
        #                     [1,2,3,4,5,7],
        #                     [1,2,3,4,5,8],
        #                     [1,2,3,4,5,9],
        #                     [1,2,3,4,5,10],
        #                     [1,2,3,4,5,11],
        #                     [1,2,3,4,5,12],
        #                     [1,2,3,4,5,13],
        #                     [1,2,3,4,5,14],
        #                     [1,2,3,4,5,15],
        #                     [1,2,3,4,5,0.01],
        #                     [1,2,3,4,5,0.02],
        #                     [1,2,3,4,5,0.03],
        #                     [1,2,3,4,5,0.04],
        #                     [1,2,3,4,5,0.05],
        #                     [1,2,3,4,5,0.16],
        #                     [1,2,3,4,5,0.17],
        #                     [1,2,3,4,5,0.18],
        #                     [1,2,3,4,5,1]
        #                     ]
        # test_arrangement = self.biorob.arrange_states(state_true)
        #


    def test_phi_arrangement(self):
        # self.setUp()
        obs = {
            'pos1': 1,
            'pos2': 2,
            'pos3': 3,
            'pos4': 4,
            'pos5': 5,
            'vel1': 6,
            'vel2': 7,
            'vel3': 8,
            'vel4': 9,
            'vel5': 10,
            'load1': 11,
            'load2': 12,
            'load3': 13,
            'load4': 14,
            'load5': 15,
            'emg1': 16,
            'emg2': 17,
            'emg3': 18,
        }

        true_arrangement = [[1,2,3,4,5,6],
                            [1,2,3,4,5,7],
                            [1,2,3,4,5,8],
                            [1,2,3,4,5,9],
                            [1,2,3,4,5,10],
                            [1,2,3,4,5,11],
                            [1,2,3,4,5,12],
                            [1,2,3,4,5,13],
                            [1,2,3,4,5,14],
                            [1,2,3,4,5,15],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 16)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 17)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 18)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 1)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 2)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 3)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 4)],
                            [1,2,3,4,5, (0 * self.biorob.decay + (1 - self.biorob.decay) * 5)],
                            ]

        state = self.biorob.get_state(obs)
        (_,test_arrangement) = self.biorob.get_phi(state)
        for i in range(len(test_arrangement)):
            print test_arrangement[i]
            for j in range(len(test_arrangement[i])):
                self.assertAlmostEqual(test_arrangement[i][j],true_arrangement[i][j])

if __name__ == '__main__':
    unittest.main()