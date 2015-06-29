#!/usr/bin/env python

import unittest
from pysrc.utilities.verifier import *
"""
python -m unittest discover
"""

class TestVerifier(unittest.TestCase):
    
    def test_one_step_instantaneous(self):
        verifier = Verifier(0, instantaneous=True)
        verifier.update(1, 2)
        self.assertAlmostEqual(verifier.update(2, 1), 0.0)
        
    def test_one_step_cumulative(self):
        verifier = Verifier(0, instantaneous=False)
        verifier.update(1, 2)
        self.assertAlmostEqual(verifier.update(2, 1), 0.0)
        
    def test_2_step_instantaneous(self):
        verifier = Verifier(0.5, instantaneous=True)
        verifier.update(reward=1, prediction=2)
        verifier.update(reward=1, prediction=1)
        self.assertAlmostEqual(verifier.update(reward=2,prediction=1), 0.0)
        
    def test_2_step_cumulative(self):
        verifier = Verifier(0.5, instantaneous=False)
        verifier.update(reward=1, prediction=3)
        verifier.update(reward=1, prediction=1)
        self.assertAlmostEqual(verifier.update(reward=2,prediction=1), 0.0)
    
if __name__=="__main__":
    
    suite = unittest.TestLoader().loadTestsFromTestCase(TestVerifier)
    unittest.TextTestRunner(verbosity=2).run(suite)