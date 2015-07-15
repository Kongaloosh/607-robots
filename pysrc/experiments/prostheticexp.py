"""
    Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
from debian.debtags import output
import os
import sys
sys.path.insert(0, os.getcwd())
import argparse
from pysrc.problems.prosthetic_problem import Prosthetic_Experiment, Prosthetic_Experiment_With_Context
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader, FileLoaderApprox
from pysrc.utilities.verifier import *
from pysrc.utilities.max_min_finder import *
import pickle
import time
import multiprocessing as mp


def runoneconfig(file_loader, alg, prob, config, out):
    """for the specific configuration, problem, alg,"""
    obs = file_loader.step()                                    # get the next observation diction
    state = prob.step(obs)                                      # initial state
    p = []                                                      # holds the predictions
    s = []                                                      # holds all of the rewards
    print ("fight the man")
    while file_loader.has_obs():                                # while we still have observations
        obs = file_loader.step()                                # get the next observation diction
        vals = prob.step(obs)                                   # state from prob
        alg.step(vals)                                          # update based on new state
        prediction = np.dot(vals['phinext'], alg.estimate())    # prediction for this time-step
        p.append(prediction)                                    # record prediction
        s.append(vals['R'])                                     # record actual reward
    #     if file_loader.i % 1000 == 0:                           # pretty print
    #         print("Step: {s} of {n}".format(s=file_loader.i, n=len(file_loader.data_stream)))
    # print("Finished: " + str((time.time()-start)/60))           # time taken for experiment
    file_loader.reset()                                          # reset the file-loader for the next exp
    config['prediction'] = np.array(p)
    config['signal'] = np.array(s)
    config['error'] = np.array(config['return']) - \
                      config['prediction'][:len(config['return'])]
    out.put(config)                                             # return the predictions and rewards


def main():
    """runs the experiment with commandline args"""
    parser = argparse.ArgumentParser()
    parser.add_argument("sVal", help="Session. single digit.")
    parser.add_argument("aVal", help="Activity value. Single digit.")
    parser.add_argument("prob", help="Name of the problem to use.")
    parser.add_argument("algname", help="name of the algorithm.")
    parser.add_argument("filename", help="name you want to add to the file")
    args = parser.parse_args()

    config_prob_path = 'results/robot-experiments/{prob}/configprob.pkl'.format(prob=args.prob)
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

    f = open('results/robot-experiments/{prob}/{alg}/{name}_{s}_{a}.dat'.format(prob=args.prob, alg=args.algname, s=args.sVal, a=args.aVal, name=args.filename), 'wb')

    # calculate the return
    calculated_return = calculate_discounted_return_backwards(config_prob,
                                                              file_loader.data_stream,
                                                              Prosthetic_Experiment)
    config_prob['return'] = calculated_return

    # calculate normalizer
    timer = time.time()
    # todo: make it so that we don't need the alg config to do this
    c = reduce(lambda x, y: dict(x, **y), (config_alg[0], config_prob)) # concat the dicts
    prob = problems[args.prob](c)                                       # construct a representative config
    config_prob['normalizer'] = generate_normalizer(file_loader.data_stream, prob=prob)                             # get constants for normalizing states

    # run the experiment
    processes = []                                  # where we define each sweep member
    output = mp.Queue()                             # where results are stored
    for config in config_alg:                       # for the parameter sweep we're interested in
        config.update(config_prob)                  # add the problem-specific configs
        try:
            config['alpha'] /= config['num_tilings']    # divide alpha
        except:
            pass                                        # we're using an alg with different config
        fl = FileLoaderApprox(
            'results/prosthetic-data/EdwardsPOIswitching_{s}{a}.txt'.format(
                s=args.sVal, a=args.aVal), 14)
        a = algs[args.algname](config)
        p = problems[args.prob](config)
        processes.append(mp.Process(target=runoneconfig, args=(fl, a, p, config, output)))

    for p in processes:
        p.start()

    for p in processes:
        p.join()

    for p in processes:
        run_result = output.get()
        pickle.dump(run_result, f, -1)


if __name__ == '__main__':
    '''from the command-line'''
    main()
