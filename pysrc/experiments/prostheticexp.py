"""
    Takes a specific algorithm and a specific file, runs across a config file looking at all the args.
"""
import os
import sys
sys.path.insert(0, os.getcwd())
import argparse
from pysrc.problems.prosthetic_problem import Experiment
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader
from pysrc.utilities.verifier import Verifier

def runoneconfig(config, file_loader, alg, prob): #todo: hook up the prob
    """for the specific configuration, problem, alg,"""
    # todo: define how we pull out the features we care about.

    while file_loader.has_obs():         # while we still have observations
        obs = file_loader.step()          # get the next observation diction
        state = prob.step(obs)
        print(state)

    """
        tiles                   ; a provided array for the tile indices to go into
        starting-element        ; first element of "tiles" to be changed (typically 0)
        num-tilings             ; the number of tilings desired
        memory-size             ; the number of possible tile indices
        floats                  ; a list of real values making up the input vector
        ints)                   ; list of optional inputs to get different hashings
    """


def main():
    """runs the experiment with commandline args"""

    # TODO: take in what alg we're running and what exp config we use
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("sVal", help="Session. single digit")
    parser.add_argument("aVal", help="Activity value. Single digit")
    parser.add_argument("algname", help="name of the algorithm")
    args = parser.parse_args()
    '''


    #TODO: actually setup the config files
    # config_prob_path = 'some path to prob'
    # config_prob = pickle.load(open(config_prob_path, 'rb'))   # we load a configuration file with all of the data
    config_prob = {'gamma':0.99, 'nf':2}

    # file_loader = FileLoader('../../results/prosthetic-data/'+data_file)
    file_loader = FileLoader('results/prosthetic-data/EdwardsPOIswitching_s1a1.txt')
    # we will only ever need to change the file name; we always navigate to the same spot

    # config_alg = picle.load(open('path/to/alg'))
    config_alg = [{'alpha':0.5, 'lambda':0.9}]

    algs  = {
        'td':td.TD,
        'totd':totd.TOTD,
        'tdr':tdr.TDR,
        'utd':utd.UTD,
        'utotd':utotd.UTOTD,
        'utdr':utdr.UTDR
    }

    # TODO: manage the results so it plugs into plotting nicely
    # f = open('where/results/go', 'wb')

    # TODO: get the verifier to calculate the return pre-exp and use that for each run
    # verifier = Verifier()

    for config in config_alg:           # for the parameter sweep we're interested in
        config.update(config_prob)      # add the problem-specific configs

        prob = Experiment(config_prob)

        # alg = algs['THING WE GOT FROM CMD LINE'](configprob)
        alg = algs['td'](config)        # build our instance of an algorithm
        runoneconfig(config=config, file_loader=file_loader, prob=prob, alg=alg,)

        # config['error']      = perf.getNormMSPVE()
        # pickle.dump(config, f, -1)


if __name__ == '__main__':
    '''from the command-line'''
    main()
