"""
    Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import argparse
from pysrc.problems.prosthetic_problem import Prosthetic_Experiment, Prosthetic_Experiment_With_Context
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader, FileLoaderApprox
from pysrc.utilities.verifier import *
from pysrc.utilities.max_min_finder import *
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
        vals = prob.step(obs)                                  # state from prob
        alg.step(vals)                                         # update based on new state
        prediction = np.dot(vals['phinext'], alg.estimate())   # prediction for this time-step
        p.append(prediction)                                    # record prediction
        s.append(vals['R'])                                    # record actual reward

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
    parser.add_argument("filename", help="name you want to add to the file")
    args = parser.parse_args()

    config_prob_path = 'results/robot-experiments/{prob}/configprob.pkl'.format(prob=args.prob)
    print(config_prob_path)
    config_prob = pickle.load(open(config_prob_path, 'rb'))   # we load a configuration file with all of the data

    config_alg_path = 'results/robot-experiments/{prob}/{alg}/configalg.pkl'.format(prob=args.prob, alg=args.algname)
    config_alg = pickle.load(open(config_alg_path, 'rb'))   # we load a configuration file with all of the data

    file_loader = FileLoaderApprox('results/prosthetic-data/EdwardsPOIswitching_{s}{a}.txt'.format(s=args.sVal, a=args.aVal), 14)
    # file_loader = FileLoader('results/prosthetic-data/EdwardsPOIswitching_{s}{a}.txt'.format(s=args.sVal, a=args.aVal))

    algs = {
        'td': td.TD,
        'totd': totd.TOTD,
        'tdr': tdr.TDR,
        'utd': utd.UTD,
        'utotd': utotd.UTOTD,
        'utdr': utdr.UTDR
    }

    problems = {
        'prosthetic_experiment': Prosthetic_Experiment,
        'prosthetic_experiment_with_context': Prosthetic_Experiment_With_Context,
    }

    ''' search for an unused file-name '''
    f = open('results/robot-experiments/{prob}/{alg}/{name}_{s}_{a}.dat'.format(prob=args.prob, alg=args.algname, s=args.sVal, a=args.aVal, name=args.filename), 'wb')

    # calculate the return
    calculated_return = \
        calculate_discounted_return_backwards(
            config_prob,
            file_loader.data_stream,
            Prosthetic_Experiment)
    config_prob['return'] = calculated_return

    # calculate normalizer
    print("normalizer start")
    timer = time.time()
    # todo: make it so that we don't need the alg config to do this
    c = reduce(lambda x, y: dict(x, **y), (config_alg[0], config_prob)) # concat the dicts
    prob = problems[args.prob](c)                                       # construct a representative config
    config_prob['normalizer'] = generate_normalizer(
        file_loader.data_stream, prob=prob)                             # get constants for normalizing states
    print("normalizer end: {time}".format(time=(time.time()-timer)))

    # run the experiment
    for config in config_alg:                       # for the parameter sweep we're interested in
        config.update(config_prob)                  # add the problem-specific configs
        try:
            config['alpha'] /= config['num_tilings']    # divide alpha
        except:
            pass                                        # we're using an alg with different config
        prob = problems[args.prob](config)                      # construct a problem
        alg = algs[args.algname](config)            # build our instance of an algorithm
        (prediction, signal) = \
            runoneconfig(file_loader=file_loader, prob=prob, alg=alg)    # grab results of run

        config['signal'] = signal                   # adding to the config so we can save results
        config['prediction'] = prediction
        pickle.dump(config, f, -1)

    pyplot.plot(calculated_return)
    pyplot.show()

if __name__ == '__main__':
    '''from the command-line'''
    main()
