
import numpy as np
import cPickle as pickle

alphas = 10**np.arange(-3, 0.1, .5)
lms = np.concatenate((np.arange(0, .9, .3), np.arange(.9, 1.01, .3)))
configs = [{'alpha': alpha, 'lmbda': lm} for alpha in alphas for lm in lms]
f = open('configalg.pkl', 'wb')

# configs = [{'alpha': 0.01, 'lmbda': 0.97}]  # should pull the alpha normalizing out eventually
f = open('configalg.pkl', 'wb')

pickle.dump(configs, f)
