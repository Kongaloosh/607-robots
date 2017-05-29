"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle
decays = [0.9]
alphas = [0.001]
lambdas = [0]

configs = [{'alpha': alpha, 'lmbda': lm, 'decay':decay}
           for alpha in alphas for lm in lambdas for decay in decays ]
print len(configs)
for i in range(len(configs)):
    f = open('configalg_{i}.pkl'.format(i=i), 'wb')
    pickle.dump([configs[i]], f)
