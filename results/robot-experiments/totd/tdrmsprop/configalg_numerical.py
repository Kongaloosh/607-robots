"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle
decays = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
alphas = [0.5]
lambdas = [0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.98, 0.99, 0.995, 0.998, 0.999, 1]

configs = [{'alpha': alpha, 'lmbda': lm, 'decay':decay}
           for alpha in alphas for lm in lambdas for decay in decays ]
print len(configs)
for i in range(len(configs)):
    f = open('configalg_{i}.pkl'.format(i=i), 'wb')
    pickle.dump([configs[i]], f)
