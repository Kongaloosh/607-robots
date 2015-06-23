'''
Generates the configuration


'''
import numpy as np
import cPickle as pickle
from math import gamma


def main():
    number_of_features = 100
    N = 1000
    gamma = 0.99
    configs = {
        # function approximation:
        'number_of_tilings': 2,
        'number of parameters': 2,
        'collision_table_size': 2**2,
        # rest
        'Gamma'      : gamma,
        'ftype'      : 'binary',
        'b'          : 10,
        'rtype'      :'uniform',
        'rparam'     : 1,
        'Rstd'       : 0.0,
        'initsdist'  : 'statezero'}

    f = open('configprob.pkl', 'wb')
    pickle.dump(configs, f)
  
if __name__ == "__main__":
  main()  
