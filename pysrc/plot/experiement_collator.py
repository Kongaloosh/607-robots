__author__ = 'alex'
"""
    to make things speedier, we take advantage of westgrid by splitting everything up and running a single
    config as an individual

"""
import os
import pickle

#todo: over all experiments
path = 'results/robot-experiments/prosthetic_experiment/'

                    #todo over all s and a vals
for alg in ['td/']: #todo: over all algs
    i = 0
    data = path+alg+"distributed_s1_a1_{i}.dat".format(i=i)
    f = open(path+alg+'total_run.dat', 'wb')
    while os.path.isfile(data):
        try:
            run_result = pickle.load(open(data,'r'))
            pickle.dump(run_result, f)
        except:
            print('file error on file {i}'.format(i=i))
        i += 1
        data = path+alg+"distributed_s1_a1_{i}.dat".format(i=i)