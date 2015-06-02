'''
Created on Mar 25, 2015

@author: A. Rupam Mahmood
'''
import unittest
import numpy as np
import os
import sys
from pysrc.experiments import rndmdpexp
import cPickle as pickle

class Test(unittest.TestCase):
  def testStdRandomWalkExp(self):
    dirpath   = "./pysrctest/experiments/rndmdpexp/deterministicnormal/"
    dirpath   = "" if not os.path.isdir(dirpath) else dirpath
    sys.argv  = ["", "1000", "1", dirpath, "totd"]
    rndmdpexp.main()
    data1        = pickle.load(open(dirpath+"totd"+"/mdpseed_1000_runseed_1.dat", "rb"))
    groundtruth1 = pickle.load(open(dirpath+"totd"+"/exp1_TOTD__mdpseed_1000_runseed_1_conf_config_normal_large_det.dat_alphaindex_0_lmbdaindex_1", "rb"))
    print data1
    print groundtruth1
    assert(abs(groundtruth1['TOTDMSPVE']-data1['error'])<10**-10)

if __name__ == "__main__":
  #import sys;sys.argv = ['', 'Test.testStdRandomWalkExp']
  unittest.main()
    