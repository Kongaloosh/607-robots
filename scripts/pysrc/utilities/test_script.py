""little tool for removing un-wanted data while collating"""
import os
import pickle

__author__ = 'alex'

keys = ['alpha', 'lmbda', 'initalpha', 'error']

path = 'results/robot-experiments/prosthetic_experiment/'
exp_name = 'experiment'
# for alg in ['td','tdr','totd','autotd']:
#     print("Collecting {alg}".format(alg=alg))
#     for s in ['s1','s2','s3','s4']:
#         print("Over subject {s}".format(s=s))
#         for a in ['a1','a2','a3','na1','na2','na3']:
#             print("Over trial {a}".format(a=a))
i = 0
data = path + "td" + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s="s1", a="a1", i="1")
# if os.path.isfile(data):
#     f = open(path + alg + '/_total_run_{s}_{a}.dat'.format(s=s, a=a), 'wb')
# while os.path.isfile(data):                     # if our s, a, or i values ar out of bounds, kick
try:
    f = open(data, 'r')
    run_result = pickle.load(f)
    dump = {}
    for key in keys:
        try:
            dump[key] = run_result[key]
        except KeyError:
            pass
    f = open(data, 'wb')
    pickle.dump(dump,f)
    f.close()
except EOFError:
    print('unexpected EOF on file {i}'.format(i=i))
    # except:
    #     print('file error on file {i}'.format(i=i))
    # i += 1
    # data = path + alg + "/{name}_{s}_{a}_{i}.dat".format(name=exp_name, s=s, a=a, i=i)