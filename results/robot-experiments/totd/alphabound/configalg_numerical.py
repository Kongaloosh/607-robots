"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle

alphas = [1]
lambdas = [0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.98, 0.99, 0.995, 0.998, 0.999, 1]
configs = [{'alpha': alpha, 'lmbda': lm}
           for alpha in alphas for lm in lambdas]
print len(configs)
for i in range(len(configs)):
    f = open('configalg_{i}.pkl'.format(i=i), 'wb')
    pickle.dump([configs[i]], f)
