"""little tool for removing un-wanted data while collating"""
import argparse
import os
import pickle

__author__ = 'alex'

keys = ['alpha', 'lmbda', 'initalpha', 'error']

path = 'results/robot-experiments/biorob/'
exp_name = 'experiment'

parser = argparse.ArgumentParser()
parser.add_argument("algs", help="Algorithms to collect over. For instance, 'autotd,totd,tdr'")
args = parser.parse_args()
algs = args.algs.split(',')

for alg in algs:
    print("Collecting {alg}".format(alg=alg))
    for s in ['s1','s2','s3','s4']:
        print("Over subject {s}".format(s=s))
        for a in ['a1','a2','a3','na1','na2','na3']:
            print("Over trial {a}".format(a=a))
            i = 0
            data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)
            while os.path.isfile(data):                     # if our s, a, or i values ar out of bounds, kick
                try:
                    f = open(data, 'r+')
                    run_result = pickle.load(f)
                    dump = {}
                    for key in keys:
                        try:
                            dump[key] = run_result[key]
                        except KeyError:
                            pass
                    f = open(data, 'wb')
                    pickle.dump(dump, f)
                    f.close()
                except EOFError:
                    print('unexpected EOF on file {i}'.format(i=i))
                i += 1
                data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)
            print("no more files.")