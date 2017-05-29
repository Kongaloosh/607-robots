__author__ = 'kongaloosh'
"""
    to make things speedier, we take advantage of westgrid by splitting everything up and running a single
    config as an individual run. This file takes the individual runs for each algorithim, each trial, and
    each subject.

    we keep things separated by trial in case someone wants to go through a specific subset of the data
"""
import argparse
import os
import pickle

__author__ = 'alex'

path = 'results/robot-experiments/totd/'
exp_name = 'experiment'
# exp_name = 'honors-pos-2016-01-16'

parser = argparse.ArgumentParser()
parser.add_argument("alg", help="Algorithms to collect over. For instance, 'autotd,totd,tdr'")
parser.add_argument("s", help="Algorithms to collect over. For instance, 'autotd,totd,tdr'")
parser.add_argument("a", help="Algorithms to collect over. For instance, 'autotd,totd,tdr'")

args = parser.parse_args()
alg = args.alg
s = args.s
a = args.a
i = 0
data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)
print "collating {0} {1} {2} {3}".format(data, s, a, i)
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
