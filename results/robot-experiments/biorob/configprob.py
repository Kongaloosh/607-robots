'''
Generates the configuration
'''

import numpy as np
import cPickle as pickle
from math import gamma


def main():
    gamma = 0.97
    configs = {
        # function approximation:
        'num_tilings': 6,
        'memory_size': 2 ** 20,
        'nf': 2 ** 20,
        'starting_element': 0,
        'gamma': gamma
    }
    f = open('configprob.pkl', 'wb')
    pickle.dump(configs, f)
  
if __name__ == "__main__":
  main()  
