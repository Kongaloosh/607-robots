"""
    to make things speedier, we take advantage of westgrid by splitting everything up and running a single
    config as an individual run. This file takes the individual runs for each algorithim, each trial, and
    each subject.
"""

import os
import pickle

__author__ = 'alex'

path = 'results/robot-experiments/prosthetic_experiment/'
exp_name = 'experiment'
for alg in ['tdr']:
    print("Collecting {alg}".format(alg=alg))
    for s in ['s1','s2','s3','s4']:
        print("Over subject {s}".format(s=s))
        for a in ['a1','a2','a3','na1','na2','na3']:
            print("Over trial {a}".format(a=a))
            i = 0
            data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)
            if os.path.isfile(data):
                f = open(path + alg + '/total_run_{s}_{a}.dat'.format(s=s, a=a), 'wb')
            while os.path.isfile(data):                     # if our s, a, or i values ar out of bounds, kick
                try:
                    run_result = pickle.load(open(data, 'r'))
                    pickle.dump(run_result, f)
                except EOFError:
                    print('unexpected EOF on file {i}'.format(i=i))
                except:
                    print('file error on file {i}'.format(i=i))
                i += 1
                data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)