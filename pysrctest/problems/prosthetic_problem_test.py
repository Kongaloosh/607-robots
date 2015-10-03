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

        self.biorob =Biorob2012Experiment(configs)
        self.prosthetic_experiment = Prosthetic_Experiment(configs)


class TestProstheticProblem (BasicProblem):
    """Ensures that the state and the rewards are correctly calculated by problem"""
    #TODO: abstract out get-phi and make sure everything's fine

    def test_get_reward_prosthetic_experiment(self):
        """Should return 1 if abs value of elbow is > 0.2 otherwise, 0"""
        self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':1}),1)
        self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':-1}),1)

        self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':0.1}),0)
        self.assertEquals(self.prosthetic_experiment.get_reward({'vel4':-0.1}),0)

    def test_get_reward_biorob(self):
        """Should return 1 if abs value of elbow is > 0.2 otherwise, 0"""
        self.assertEquals(self.biorob.get_reward({'vel4':1}),1)
        self.assertEquals(self.biorob.get_reward({'vel4':-1}),1)

        self.assertEquals(self.biorob.get_reward({'vel4':0.1}),0)
        self.assertEquals(self.biorob.get_reward({'vel4':-0.1}),0)

    def test_get_state_prosthetic_experiment(self):
        """Should only return the states which are not labeled random_val_#"""
        obs = {
            'random_val_1': 100,
            'pos1': 1,
            'random_val_2': 99,
            'pos2': 2,
            'random_val_3': 98,
            'pos3': 3,
            'random_val_4': 97,
            'pos5': 4,
            'random_val_5': 96,
            'vel1': 5,
            'random_val_6': 95,
            'vel2': 6,
            'random_val_7': 94,
            'vel3': 7,
            'random_val_8': 93,
            'vel5': 8,
            'random_val_9': 92,
            'load5': 9,
            'random_val_9': 91,
        }
        state_true = [1,2,3,4,5,6,7,8,9]
        state = self.prosthetic_experiment.get_state(obs)
        self.assertItemsEqual(state, state_true)

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

        # state_true = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        #               (0 * self.biorob.decay + (1 - self.biorob.decay) * 1),
        #               (0 * self.biorob.decay + (1 - self.biorob.decay) * 2),
        #               (0 * self.biorob.decay + (1 - self.biorob.decay) * 3),
        #               (0 * self.biorob.decay + (1 - self.biorob.decay) * 4)]

        print("1")
        state = self.biorob.step(obs)
        # self.assertItemsEqual(state, state_true)
        # print([i for i, e in enumerate(state['phinext']) if e != 0])
        # print([i for i, e in enumerate(state['phi']) if e != 0])



        # print("2")
        # state = self.biorob.step(obs)
        # # self.assertItemsEqual(state, state_true)
        # print([i for i, e in enumerate(state['phinext']) if e != 0])
        # print([i for i, e in enumerate(state['phi']) if e != 0])

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

        print("3")
        state = self.biorob.step(obs)
        # self.assertItemsEqual(state, state_true)
        # print([i for i, e in enumerate(state['phi']) if e != 0])
        # print([i for i, e in enumerate(state['phinext']) if e != 0])



if __name__ == '__main__':
    unittest.main()