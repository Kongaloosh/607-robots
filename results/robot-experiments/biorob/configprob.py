
'''
Generates the configuration
'''

import numpy as np
import cPickle as pickle
from math import gamma


def main():
    configs = {
        # function approximation:
        'num_tilings': 8,
        'memory_size': 2 ** 20,
        'nf': 2 ** 20,
        'starting_element': 0,
        'gamma': 0.97             # What I was using
        # 'gamma': 0.992              # From biorob 2012
    }
    f = open('configprob.pkl', 'wb')
    pickle.dump(configs, f)
  
if __name__ == "__main__":
  main()  
