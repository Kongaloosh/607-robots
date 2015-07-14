import pickle
import numpy as np
from matplotlib import pyplot

__author__ = 'alex'

a = pickle.load(open('results/robot-experiments/prosthetic_experiment_with_context/tdr/testing_s1_a1.dat'))

pyplot.plot(a['prediction'])
pyplot.plot(np.array(a['signal'])*33.3333)                         # ploting for observation
pyplot.plot(a['return'])
pyplot.show()

pyplot.plot(np.array(a['signal']))
pyplot.plot(np.array(a['prediction'])*(1-a['gamma']))
pyplot.show()
