"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle
import numpy as np

meta_step_size  = [0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009, 0.01, 0.125, 0.15, 0.02]
lambdas         = [0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.98, 0.99, 0.995, 0.998, 0.999, 1]
betas           = np.log([0.05, 0.1, 0.2, 0.3, 0.6, 1.0, 1.5, 2.0])
configs         = [{'meta_step_size': step_size, 'lmbda': lm, 'beta': beta}
           for step_size in meta_step_size for lm in lambdas for beta in betas]
print len(configs)
for i in range(len(configs)):
    f = open('configalg_{i}.pkl'.format(i=i), 'wb')
    pickle.dump([configs[i]], f)
