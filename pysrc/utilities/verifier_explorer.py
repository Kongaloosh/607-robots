from pysrc.utilities.verifier import *
import numpy
from matplotlib import pyplot

__author__ = 'alex'

class spoof(object):                    # we create a spoofed 'problem' to pass in and calc rewards

    def __init__(self, config):
        pass

    @staticmethod
    def get_reward(obs):
        return 1

config = {'gamma': 0.9}
obs = numpy.zeros(20000)
ret = calculate_discounted_return_backwards(config, obs, spoof)

pyplot.plot(ret)
pyplot.show(globals())

