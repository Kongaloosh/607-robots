"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle

# alphas = 10**np.arange(-3, 0.1, .25)                                         #values for alpha sweep-over
# lambdas = np.concatenate((np.arange(0, .9, .1), np.arange(.9, 1.01, .01)))    #values of lambda to sweep over
#
# configs = [{'alpha': alpha, 'lmbda': lm} for alpha in alphas for lm in lambdas]
configs = [{'alpha': 0.1, 'lambda': 0.5}]

f = open('configalg.pkl', 'wb')

pickle.dump(configs, f)
