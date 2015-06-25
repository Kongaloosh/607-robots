"""

"""
import os
import sys
sys.path.insert(0, os.getcwd())
import argparse
from pysrc.algorithms.tdprediction.onpolicy import td, tdr, totd, utd, utotd, utdr
from pysrc.utilities.file_loader import FileLoader
from pysrc.utilities.tiles import loadTiles, CollisionTable


def runoneconfig(config, file_loader, alg, perf): # perf would be a verifier
    """for the specific configuration, problem, alg,"""
    parameters = {'l':config['lambda'],
                  'lnext':config['lambda'],
                  'g':config['gamma'],
                  'gnext':config['gamma']}  # we presume that for the time being, these don't change

    obs = []                            # where we put the observation
    while file_loader.hasObs():         # while we still have observations
        f = file_loader.step()          # get the next observation diction
        for k in f:                     # for all the key values...
            obs.append(f[k])            # append to our current obs

        """
               tiles                   ; a provided array for the tile indices to go into
               starting-element        ; first element of "tiles" to be changed (typically 0)
               num-tilings             ; the number of tilings desired
               memory-size             ; the number of possible tile indices
               floats                  ; a list of real values making up the input vector
               ints)                   ; list of optional inputs to get different hashings
        """

        loadTiles(config['phinext'], 0, config['num-tilings'], memctable=config['memory-size'], floats=k)

        if f['val'] == 1:               # if the joint is active
            parameters['gnext'] = 1     # consider this to be the end of the trial
        else:
            parameters['gnext'] = config['gamma']   # otherwise, reg gamma

        parameters['R'] = f['target']
        """ phi, 'R', phinext, g, l, gnext """

        alg.step(parameters)
        perf.calcMSPVE(alg, parameters['R'])

        # TEAR-DOWN
        parameters['phi'] = parameters['phinext']
        parameters['g'] = parameters['gnext']



def main():
    """runs the experiment with commandline args"""
    parser = argparse.ArgumentParser()
    parser.add_argument("sVal", help="Session. single digit")
    parser.add_argument("aVal", help="Activity value. Single digit")
    parser.add_argument("algname", help="name of the algorithm")
    args = parser.parse_args()

    #TODO ADD SOME FILE HANDLER THAT TAKES CARE OF LOADING EXP FILES
    configAlgPathName = "results/robot-experiments/prosthetic-data/" + \
                        "EdwardsPOIswitching_s1a1.txt" + \
                        "s" + args.sVal + \
                        "a" + args.aVal + ".txt"
    obs = FileLoader(configAlgPathName)

    #TODO ADD SOME FILE HANDLER THAT TAKES CARE OF LOADING ALG FILES
    algs  = {
        'td':td.TD,
        'totd':totd.TOTD,
        'tdr':tdr.TDR,
        'utd':utd.UTD,
        'utotd':utotd.UTOTD,
        'utdr':utdr.UTDR
    }
    # TODO: figure out whether or not we want to iterate over all the files
    # for file in sessions:
    # configure the alg
    # run one alg
    # add the verifiers NORM MSPVE to the 'error'
    # dump values

if __name__ == '__main__':
    '''from the command-line'''
    main()
