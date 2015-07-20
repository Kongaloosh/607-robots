"""
Created on Jan 25, 2015

@author: A. Rupam Mahmood

Responsibility of this program is to take several output files
from different runs for a particular algorithm on an experiment
and extract simple tables containing performance vs. a given
set of parameter by averaging over the runs and choosing the
best values for the rest of the parameters. This table can then
be easily loaded up and plotted using standard matplotlib tools.
"""

import numpy as np
import cPickle as pickle
import itertools
import sys
import os.path


def loaddata(nruns, pathfileprefix):
    """
    Takes in the number of runs---or, files we have---and looks for all the files
    for any given file, we load all of the results and find.

    Remember, the results are going to be a collection of pickled items, each
    being one member of a sweep for a particular episode.

    :param nruns: number of runs
    :param pathfileprefix: the prefix to where we want to find result files

    """
    data = []
    for run in range(1, nruns + 1):                         # for all the possible runs
        filepathname = pathfileprefix + ".dat"              # point to data file
        f = open(filepathname, 'rb')                        # load the file results are stored in
        try:                                                # While we still have sweep results in a given file
            while True:                                     # load and append the data to our array
                d = pickle.load(f)                          # load and append a run
                data.append(d)
        except EOFError:                                    # catch the end of the file to move on
            print 'End of file reached'
    return data                                             # return all the possible files


def createtable(data, params, neps):
    """
    returns a table where
    :param data: ????
    :param params: typically alpha, lambda
    :param neps: number of episodes
    :returns table: a matrix where each element is a different run with a lambda alpha combination with a listed error.
    """
    table = np.zeros((len(data), len(params) + neps))       # make a multid array; datalength X parameters+episodes
    nparams = len(params)                                   # set number of params as len params
    for i in range(len(data)):                              # for i->data
        for j in range(nparams):                            # for all the parameters (alpha/lambda)
            table[i, j] = data[i][params[j]]                # for a data entry get alpha or lambda
        table[i, nparams:] = sum(abs(data[i]['error']))     # for a run get the error and put it in the table
    return table                                            # return the data structure

# TODO check
def createtablelearningcurves(table, num_runs, num_eps=1):
    """
    :param table: parameter combination by
    :param num_runs: the number of runs in the file
    :param num_eps: the number of episodes. For prosthetics this is always 1
    """
    (tablerows, tablecols)      = np.shape(table)                       # create matrix from table shape
    tableavgrows                = tablerows / num_runs
    nparams                     = tablecols - num_eps                   # the number of parameters we compare over
    tableavg                    = np.zeros((tableavgrows, tablecols))   #
    tablestd                    = np.zeros((tableavgrows, tablecols))   #
    tabletemp                   = np.zeros((tableavgrows, num_eps))     #
    tableavg[:, :nparams]       = table[:tableavgrows, :nparams]        #
    tablestd[:, :nparams]       = table[:tableavgrows, :nparams]        #
    tabletemp[:, :num_eps]      = table[:tableavgrows, nparams:]        #

    for i in range(1, num_runs):                                        #
        tabletemp = np.concatenate(
            (
                tabletemp,                                                  #
                table[i * tableavgrows: (i + 1) * tableavgrows, nparams:]   #
            ),
            )
    #print np.shape(table[(i)*tableavgrows:(i+1)*tableavgrows, nparams:])
    tableavg[:, nparams:] = np.mean(np.reshape(tabletemp, (tableavgrows, num_runs, num_eps)), 1)
    tablestd[:, nparams:] = np.std(np.reshape(tabletemp, (tableavgrows, num_runs, num_eps)), 1) / np.sqrt(num_runs)

    return (tableavg, tablestd)


def createtableavg(table, nruns, neps, startstep=0):
    """
    Returns the table with the mean error and the deviation of the error over the sqrt of the runs
    :param table: a table which contains the error and the parameters for a sweep element
    :param nruns: the number of run
    :param neps: the number of episodes for a particular sweep
    :param startstep: for our purposes this will always be 0
    """
    (tablerows, tablecols) = np.shape(table)                    # tuple which matches data
    tableavgrows = tablerows / nruns                            # average over the runs
    nparams = tablecols - neps                                  # the number of compared parameters
    tableavgcols = nparams + 2                                  # ???
    tableavgstd = np.zeros((tableavgrows, tableavgcols))        # the table we put our averages into
    tableavgstd[:, :nparams] = table[:tableavgrows, :nparams]   # add the parameter values
    tabletemp = table[:tableavgrows, (nparams + startstep):]    # the error for each run
    for i in range(1, nruns):                                   # since the number of runs is one it will
        tabletemp = np.concatenate(                             # never evaluate
            (
                tabletemp,
                table[i * tableavgrows:(i + 1) * tableavgrows, (nparams + startstep):]
            ),
            1
        )
    r, c = np.shape(tabletemp)
    tabletemp2 = np.mean(np.reshape(tabletemp, (r, c / nruns, nruns), 1), 1)    #
    tableavgstd[:, nparams] = np.mean(tabletemp2, 1)                            # the mean of one element
    tableavgstd[:, nparams + 1] = np.std(tabletemp2, 1) / np.sqrt(nruns)        #
    return tableavgstd

# todo: check
def performancevsparams(tableavgstd, params, paramssub):
    """

    :param tableavgstd: the averaged table
    :param params: the parameters we vary over. Ex param
    :param paramssub: the parameter we compare against. Ex. Lambda
    """
    (tableavgrows, tableavgcols) = np.shape(tableavgstd)                # The dimensions of the avg'd table
    paramvals = {}                                                      # where we put our parameter values
    nparamssubvals = 1.                                                 # number of parameters we compare about
    for param in params:                                                #
        paramvals[param] = np.unique(tableavgstd[:, param == params])   #
        print paramvals[param]
        if (param == paramssub).any():                                  #
            nparamssubvals *= len(paramvals[param])                     #

    paramsubvalcomblist = list(
        itertools.product(*[paramvals[param] for param in paramssub])
    )

    perftable = np.zeros(
        (
            len(paramsubvalcomblist),
            tableavgcols - len(params) + len(paramssub)
        )
    )
    row = 0
    # print(paramsubvalcomblist)
    for paramsubvalcomb in paramsubvalcomblist:
        paramsubvalcomb = np.array(paramsubvalcomb)
        condition = np.array(np.repeat(True, tableavgrows))
        # print(paramsubvalcomb)
        for param in params:
            # Todo: figure out what this is
            if (param == paramssub).any():
                condition = condition * (
                    tableavgstd[:, param == params] == paramsubvalcomb[param == paramssub]).reshape(tableavgrows)

        perftable[row, :len(paramssub)] = paramsubvalcomb
        perftable[row, len(paramssub)] = np.nanmin(tableavgstd[condition, len(params)])
        perftable[row, len(paramssub) + 1] = np.nanmin(tableavgstd[condition, len(params) + 1])
        row += 1
    return perftable


def main():
    """ plotter needs cmdline args """
    if len(sys.argv) > 1:
        num_runs = int(sys.argv[1])                                     # number of runs
        pathfile_prefix = sys.argv[2]                                   # prefix of all the files we look at
        num_params = int(sys.argv[3])                                   # the number of parameters
        params = np.array([sys.argv[4+i] for i in range(num_params)])   # an array with parameters alpha lambda listed
    tablefilename = pathfile_prefix + "perftable.plot.pkl"              # make a file for the table
    if not os.path.isfile(tablefilename):                               # if the table doesn't exist already
        data = loaddata(num_runs, pathfile_prefix)                      # Produce and dump the averaged table first
        # neps = data[0]['N']                                           # number of episodes
        neps = 1
        table = createtable(data, params, neps)                         # get a table comparing lambda and alpha
        tableavgstd = createtableavg(table, num_runs, neps)             # average the tables
        fs = open(tablefilename, "wb")                                  # open a file to dump the average
        pickle.dump(tableavgstd, fs)                                    # dump the average
    else:
        tableavgstd = pickle.load(open(tablefilename, "rb"))            # if the table exists, load it

    # todo: make this fail noisey
    if len(sys.argv) == 4 + num_params:                                 # if we don't have enough arguments
        return                                                          # fail

    # If a subset of parameters provided, then produce a table with respect to those parameters
    nparamssub = int(sys.argv[4 + num_params])
    paramssub = np.array([sys.argv[4 + num_params + 1 + i] for i in range(nparamssub)])
    perf_table = performancevsparams(tableavgstd, params, paramssub)    # records the performance across all runs
    fsname = pathfile_prefix + 'perfvs'                                   # where calculated performance goes
    for i in range(len(paramssub)):
        fsname += paramssub[i]

    fsname += ".plot.pkl"
    fs = open(fsname, "wb")
    pickle.dump(perf_table, fs)

if __name__ == '__main__':
    main()
