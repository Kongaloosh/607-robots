"""
    Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import argparse
from pysrc.problems.prosthetic_problem import Experiment
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader
from pysrc.utilities.verifier import Verifier
import pickle
from matplotlib import pyplot
import time
def runoneconfig(config, file_loader, alg, prob, verifier): #todo: hook up the prob
    """for the specific configuration, problem, alg,"""
    # todo: define how we pull out the features we care about.
    obs = file_loader.step()          # get the next observation diction
    state = prob.step(obs)
    l = []
    p = []
    r = []
    s = []
    i = 0
    
    print("start")
    start = time.time()
    while file_loader.has_obs():         # while we still have observations
        # start = time.time()
        obs = file_loader.step()          # get the next observation diction
        # print ("=> Obs time " + str(time.time()-start))
        if file_loader.i % 10000 == 0:
            print(str("num steps: ") + str(file_loader.i))

        if file_loader.i % 14 == 0:
            # todo: tighten up all of this
            state = prob.step(obs)
            alg.step(state)
            prediction = np.dot(state['phinext'], alg.estimate()) * (1-state['gnext'])
            e = verifier.update(state['R'], prediction)
            ret = verifier.calculate_predicted_return()
            if prediction:
                p.append(prediction)
            if e:
                l.append(e)
                r.append(ret)
            s.append(state['R'])
    print("Finished: " + str(start-time.time()))
    return l,p,r,s

def main():
    """runs the experiment with commandline args"""
    parser = argparse.ArgumentParser()
    # TODO: get the problem from the command-line
    parser.add_argument("sVal", help="Session. single digit.")
    parser.add_argument("aVal", help="Activity value. Single digit.")
    parser.add_argument("prob", help="Name of the problem to use.")
    parser.add_argument("algname", help="name of the algorithm.")
    args = parser.parse_args()

    config_prob_path = 'results/robot-experiments/{prob}/configprob.pkl'.format(prob=args.prob)
    config_prob = pickle.load(open(config_prob_path, 'rb'))   # we load a configuration file with all of the data

    config_alg_path = 'results/robot-experiments/{prob}/{alg}/configalg.pkl'.format(prob=args.prob, alg=args.algname)
    config_alg = pickle.load(open(config_alg_path, 'rb'))   # we load a configuration file with all of the data

    file_loader = FileLoader('results/prosthetic-data/EdwardsPOIswitching_s{s}a{a}.txt'.format(s=args.sVal, a=args.aVal))

    algs = {
        'td': td.TD,
        'totd': totd.TOTD,
        'tdr': tdr.TDR,
        'utd': utd.UTD,
        'utotd': utotd.UTOTD,
        'utdr': utdr.UTDR
    }

    # TODO: manage the results so it plugs into plotting nicely
    i = 0
    while os.path.isfile('results/robot-experiments/{prob}/{alg}/{i}'.format(prob=args.prob, alg=args.algname, i=i)):
        i += 1
    f = open('results/robot-experiments/{prob}/{alg}/{i}'.format(prob=args.prob, alg=args.algname, i=i), 'wb')

    # TODO: get the verifier to calculate the return pre-exp and use that for each run
    verifier = Verifier(config_prob['gamma'])
    # verifier.

    for config in config_alg:           # for the parameter sweep we're interested in
        config.update(config_prob)      # add the problem-specific configs
        prob = Experiment(config)

        alg = algs[args.algname](config)        # build our instance of an algorithm
        (error, prediction, ret, signal) = runoneconfig(config=config, file_loader=file_loader, prob=prob, alg=alg, verifier=verifier)
        config['error'] = error

        pickle.dump(config, f, -1)
        pickle.dump((error,prediction,ret,signal), open('things','wb'))
        # pyplot.plot(np.abs(np.array(error)))
        pyplot.plot(np.array(prediction))
        # pyplot.plot(np.array(ret))
        pyplot.plot(np.array(signal))
        pyplot.show()
        pickle.dump((error,prediction,ret,signal), open('things_a','wb'))

if __name__ == '__main__':
    '''from the command-line'''
    main()
