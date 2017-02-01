from verifier import calculate_discounted_return_backwards
import numpy
from matplotlib import pyplot

__author__ = 'alex'

class spoof(object):                    # we create a spoofed 'problem' to pass in and calc rewards
    def __init__(self, config):
        pass

    @staticmethod
    def get_reward(obs):
        if obs <=2000:
            return 1
        else:
            return 0

config = {'gamma': 0.9}
obs = numpy.zeros(20000)
for i in range(len(obs)):
    obs[i] = i
ret = calculate_discounted_return_backwards(config, obs, spoof)

pyplot.plot(numpy.array(ret) * (1 - 0.9))
pyplot.show(globals())

