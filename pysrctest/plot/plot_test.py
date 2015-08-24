import unittest
from pysrc.plot.plot import find_best_parameters
__author__ = 'alex'


class BasicPlot(unittest.TestCase):
    """defines common setup"""

    def setUp(self):
        self.data = {
            '1': {
                '1': {'error': 0, 'std': 0},
                '2': {'error': 1, 'std': 0},
                '3': {'error': 2, 'std': 0}
            },
            '2': {
                '1': {'error': 1, 'std': 0},
                '2': {'error': 2, 'std': 0},
                '3': {'error': 3, 'std': 0}
            },
            '3': {
                '1': {'error': 2, 'std': 0},
                '2': {'error': 3, 'std': 0},
                '3': {'error': 4, 'std': 0}
            }
        }


class TestPlotter(BasicPlot):
    """Ensures that the minimum found for each val is correct"""

    def test_get_reward_prosthetic_experiment(self):
        """Should return 1 if abs value of elbow is > 0.2 otherwise, 0"""
        minimum = find_best_parameters(self.data)
        true_minimum = [(1, 0, 0), (2, 1, 0), (3, 2, 0)]
        self.assertItemsEqual(minimum, true_minimum)

if __name__ == '__main__':
    unittest.main()