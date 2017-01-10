"""
Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
import os
import sys
sys.path.insert(0, os.getcwd())
import argparse
from pysrc.problems.prosthetic_problem import Prosthetic_Experiment, Biorob2012Experiment, TOTDExperiment, TOTD_kanerva
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, autotd, dasautotd
from pysrc.utilities.file_loader import FileLoader, FileLoaderApprox, FileLoaderSetEnd
from pysrc.utilities.verifier import *
from pysrc.utilities.max_min_finder import *
import pickle
import numpy


def run_one_config(file_loader, alg, prob):
    """for the specific configuration, problem, alg, do a full run over our selected data file"""
    obs = file_loader.step()                                    # get the next observation diction
    prob.step(obs)                                              # initial step
    p = []                                                      # holds the predictions
    s = []                                                      # holds all of the rewards
    while file_loader.has_obs():                                # while we still have observations
        obs = file_loader.step()                                # get the next observation diction
        vals = prob.step(obs)                                   # state from prob
        alg.step(vals)                                          # update based on new state
        s.append(vals['R'])                                     # record actual reward
        try:
            p.append(numpy.dot(vals['phinext'], alg.estimate()))    # record the prediction
        except:
            p.append(alg.estimate(vals['phinext']))
        # if file_loader.i % 1000 == 0:                           # pretty print
        #     print(numpy.dot(vals['phinext'], alg.estimate()))
        #     print("Step: {s} of {n}".format(s=file_loader.i, n=len(file_loader.data_stream)))
            # print(vals['R'])
    file_loader.reset()                                        # sets the file-loader to obs 0 for next run
    return p, s                                                 # return the predictions and rewards


def main():
    """runs the experiment with commandline args"""
    # ==================================================================================================================
    #                                              FILE PARSING AND EXP SET-UP
    # ==================================================================================================================
    parser = argparse.ArgumentParser()
    parser.add_argument("sVal", help="Session. single digit.")
    parser.add_argument("aVal", help="Activity value. Single digit.")
    parser.add_argument("prob", help="Name of the problem to use.")
    parser.add_argument("algname", help="name of the algorithm.")
    parser.add_argument("filename", help="name you want to add to the file")
    parser.add_argument('config_number', type=int, help="the number of the desired config file. if none, then unumbered is used")
    args = parser.parse_args()


    config_prob_path = 'results/robot-experiments/{prob}/configprob.pkl'.format(prob=args.prob)
    config_prob = pickle.load(open(config_prob_path, 'rb'))   # we load a configuration file with all of the data
    num = args.config_number
    if num != -1:
        config_alg_path = \
            'results/robot-experiments/{prob}/{alg}/configalg_{i}.pkl'.format(prob=args.prob, alg=args.algname, i=num)
    else:
        config_alg_path = \
            'results/robot-experiments/{prob}/{alg}/configalg.pkl'.format(prob=args.prob, alg=args.algname)

    config_alg = pickle.load(open(config_alg_path, 'rb'))   # we load a configuration file with all of the data
    file_loader = FileLoaderSetEnd('results/prosthetic-data/EdwardsPOIswitching_{s}{a}.txt'.format(s=args.sVal, a=args.aVal), 58000)

    algs = {
        'autotd': autotd.AutoTD,
        'td': td.TD,
        'totd': totd.TOTD,
        'dasautotd': dasautotd.TD,
        'dasautotdr': dasautotd.TDR,
        'dasautotdr_kanerva': dasautotd.TDR_Kanerva,
        'tdr': tdr.TDR,
        'tdr_kanerva': tdr.TDR_Kanerva,
        'tdr_mgd': tdr.TDR_MGD,
       # 'utd': utd.UTD,
       # 'utotd': utotd.UTOTD,
       # 'utdr': utdr.UTDR
    }                                                                                   # To handle creation of alg

    problems = {
        'prosthetic_experiment': Prosthetic_Experiment,
        'biorob': Biorob2012Experiment,
        'totd': TOTDExperiment,
        'totd_kanerva': TOTD_kanerva
    }                                                                                   # To handle creation of problem

    f = open('results/robot-experiments/{prob}/{alg}/{name}_{s}_{a}_{i}.dat'.format(
        prob=args.prob,
        alg=args.algname,
        s=args.sVal,
        a=args.aVal,
        name=args.filename,
        i=args.config_number), 'wb')                                                    # where we'll write results

    calculated_return = calculate_discounted_return_backwards(
        config_prob,
        file_loader.data_stream,
        problems[args.prob]
    )                                                                                   # calculate this file's return

    config_prob['return'] = calculated_return


    c = reduce(lambda x, y: dict(x, **y), (config_alg[0], config_prob))                 # concat the dicts
    prob = problems[args.prob](c)                                                       # construct a config
    config_prob['normalizer'] = generate_normalizer(file_loader.data_stream, prob=prob) # array for normalizing states

    # ==================================================================================================================
    #                                              RUN EXPERIMENT
    # ==================================================================================================================

    for config in config_alg:                                               # for the parameters we're interested in
        configNumFeatures = 0

        if config['nf']:
            configNumFeatures = config['nf']

        config.update(config_prob)

        if configNumFeatures > 0:                                          # add the problem-specific configs
            config['nf'] = configNumFeatures
            config['memory_size'] = configNumFeatures

        prob = problems[args.prob](config)                                  # construct a problem
        config['active_features'] =\
            prob.get_num_active_features(file_loader.data_stream[0])
        alg = algs[args.algname](config)                                    # build our instance of an algorithm
        print(config)
        (prediction, signal) = \
            run_one_config(file_loader=file_loader, prob=prob, alg=alg)     # grab results of run
        config['signal'] = signal                                           # adding the config so we can save results
        config['prediction'] = prediction
        config['error'] = \
            np.array(config['return']) - prediction[:len(config['return'])]
        pickle.dump(config, f, -1)

    f.close()

if __name__ == '__main__':
    """from the command-line"""
    main()
