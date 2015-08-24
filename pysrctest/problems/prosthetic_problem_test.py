import unittest
from pysrc.problems.prosthetic_problem import Prosthetic_Experiment, Biorob2012Experiment


class BasicProblem (unittest.TestCase):
    """defines common setup"""

    def setUp(self):
        configs = {
            'num_tilings':0,
            'memory_size':0,
            'gamma':0,
            'lmbda':0,
            'normalizer':[0,0,0,0]
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
            'load1': 9,
            'random_val_9': 91,
            'load2': 10,
            'random_val_9': 91,
            'load3': 11,
            'random_val_9': 91,
            'load5': 12,
            'random_val_9': 91,
        }

        state_true = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 1),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 2),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 3),
                      (0 * self.biorob.decay + (1 - self.biorob.decay) * 4)]
        state = self.biorob.get_state(obs)

        self.assertItemsEqual(state, state_true)


if __name__ == '__main__':
    unittest.main()