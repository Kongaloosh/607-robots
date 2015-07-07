"""
    Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import argparse
from pysrc.problems.prosthetic_problem import Experiment, Experiment_With_Context
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader, FileLoaderApprox
from pysrc.utilities.verifier import *
import pickle
from matplotlib import pyplot
import time


def runoneconfig(file_loader, alg, prob):
    """for the specific configuration, problem, alg,"""
    obs = file_loader.step()                                    # get the next observation diction
    state = prob.step(obs)                                      # initial state
    p = []                                                      # holds the predictions
    s = []                                                      # holds all of the rewards

    print("start")
    start = time.time()                                         # experiment timer

    while file_loader.has_obs():                                # while we still have observations
        obs = file_loader.step()                                # get the next observation diction
        state = prob.step(obs)                                  # state from prob
        alg.step(state)                                         # update based on new state
        prediction = np.dot(state['phinext'], alg.estimate())   # prediction for this time-step
        p.append(prediction)                                    # record prediction
        s.append(state['R'])                                    # record actual reward

        if file_loader.i % 1000 == 0:                           # pretty print
            print("Step: {s} of {n}".format(s=file_loader.i, n=len(file_loader.data_stream)))

    print("Finished: " + str((time.time()-start)/60))           # time taken for experiment
    return p, s                                                 # return the predictions and rewards


def main():
    """runs the experiment with commandline args"""

    ''' Argument parsing from the command line '''

    parser = argparse.ArgumentParser()
    parser.add_argument("sVal", help="Session. single digit.")
    parser.add_argument("aVal", help="Activity value. Single digit.")
    parser.add_argument("prob", help="Name of the problem to use.")
    parser.add_argument("algname", help="name of the algorithm.")
    args = parser.parse_args()

    config_prob_path = 'results/robot-experiments/{prob}/configprob.pkl'.format(prob=args.prob)
    config_prob = pickle.load(open(config_prob_path, 'rb'))   # we load a configuration file with all of the data

    config_alg_path = 'results/robot-experiments/{prob}/{alg}/configalg.pkl'.format(prob=args.prob, alg=args.algname)
    config_alg = pickle.load(open(config_alg_path, 'rb'))   # we load a configuration file with all of the data

    file_loader = FileLoaderApprox('results/prosthetic-data/EdwardsPOIswitching_{s}{a}.txt'.format(s=args.sVal, a=args.aVal), 14)

    algs = {
        'td': td.TD,
        'totd': totd.TOTD,
        'tdr': tdr.TDR,
        'utd': utd.UTD,
        'utotd': utotd.UTOTD,
        'utdr': utdr.UTDR
    }

    ''' search for an unused file-name '''
    i = 0
    while os.path.isfile('results/robot-experiments/{prob}/{alg}/{s}_{a}-{i}.dat'.format(prob=args.prob, alg=args.algname, s=args.sVal, a=args.aVal, i=i)):
        i += 1
    f = open('results/robot-experiments/{prob}/{alg}/{s}_{a}-{i}.dat'.format(prob=args.prob, alg=args.algname, s=args.sVal, a=args.aVal, i=i), 'wb')

    ''' calculate return '''

    print("ver start")
    timer = time.time()
    print(len(file_loader.data_stream))
    calculated_return = calculate_discounted_return_backwards(config_prob, file_loader.data_stream, Experiment)
    config_prob['return'] = calculated_return
    print("ver end: {time}".format(time=(time.time()-timer)))
    pyplot.plot(calculated_return)

    for config in config_alg:                       # for the parameter sweep we're interested in
        config.update(config_prob)                  # add the problem-specific configs
        prob = Experiment_With_Context(config)      # construct a problem
        alg = algs[args.algname](config)            # build our instance of an algorithm
        (prediction, signal) = \
            runoneconfig(file_loader=file_loader, prob=prob, alg=alg)    # grab results of run

        config['signal'] = signal                   # adding to the config so we can save results
        config['prediction'] = prediction
        pickle.dump(config, f, -1)

        pyplot.plot(prediction)
        pyplot.plot(np.array(signal)*33.3333)                         # ploting for observation
        pyplot.plot(calculated_return)
        pyplot.show()

        pyplot.plot(np.array(signal))
        pyplot.plot(np.array(prediction)*(1-config['gamma']))
        pyplot.show()
if __name__ == '__main__':
    '''from the command-line'''
    main()
