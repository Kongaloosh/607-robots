"""specifies the configuration for the algorithms"""
import numpy as np
import cPickle as pickle

meta_step_size  = [0]
lambdas = [0.998]
beta            = np.log(0.01)
configs         = [{'meta_step_size': step_size, 'lmbda': lm, 'beta': beta}
                       for step_size in meta_step_size for lm in lambdas]

print len(configs)
f = open('configalg.pkl', 'wb')
pickle.dump(configs, f)
