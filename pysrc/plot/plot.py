import os
import sys
sys.path.insert(0, os.getcwd())
from matplotlib import pyplot
import cPickle as pickle
from itertools import chain
import numpy

def plot_performance_vs_lambda(path_prefix, label, parameters):
    plot_file_name = path_prefix + "perf_vs_lambda" + ".plot.pkl"
    if not os.path.isfile(plot_file_name):  # if we haven't made a plot file yet
        plot_data_process_prosthetic(parameters, path_prefix)
    # plotfile = file(plot_file_name, "rb")
    # data = pickle.load(plotfile)
    # # print(data[:, 2])
    # ppl.plot(data[:, 2])
    # ppl.errorbar(data[:,0], data[:,1], data[:,2], label=label)


def plot_data_process_prosthetic(parameters, path_prefix):
    """
    We construct a table against a particular parameter
    :param parameters: a dictionary as seen in main()
    :param path_prefix: where the path goes to
    """
                                                                    # the different values we compare performance across
    comparison_val = parameters['compare']                          # the name of the key for the variable we compare across
    data = loaddata(path_prefix)                                    # we load all the runs from a trial
    comparison_values = set([run[comparison_val] for run in data])  # we find the values we wish to vary over

    plot_points = []
    for run_parameter in comparison_values:
        best = None
        best_error = numpy.infty
        for run in data:
            new_error = sum(abs(run['error']))
            if run[comparison_val] == run_parameter and new_error < best_error:
                best_error = new_error
                best = run
        print(best_error)
        plot_points.append(best)


def loaddata(path, nruns = 1):
    """
    Takes in the number of runs---or, files we have---and looks for all the files
    for any given file, we load all of the results and find.

    Remember, the results are going to be a collection of pickled items, each
    being one member of a sweep for a particular episode.

    :param nruns: number of runs
    :param path: the prefix to where we want to find result files

    """
    data = []
    for run in range(1, nruns + 1):                         # for all the possible runs
        # todo: change number of runs over file names!!!
        filepathname = path + ".dat"                        # point to data file
        f = open(filepathname, 'rb')                        # load the file results are stored in
        try:                                                # While we still have sweep results in a given file
            while True:                                     # load and append the data to our array
                d = pickle.load(f)                          # load and append a run
                data.append(d)
        except EOFError:                                    # catch the end of the file to move on
            print 'End of file reached'
    return data


def main():
    """
    Constructs a file comparing the algs across for each alpha
    :returns none:
    """
    # todo: factor all of the stuff out. we should be able to tell parameters by algs
    path = "results/robot-experiments/biorob/"          # the path to the experiments
    postfix = 'sweep_pool_2_s1_na1'                     # the name of the experiment run
    # if not os.path.exists(path):
    #     path = "../." + path

    pathfileprefix = path+"td/"+postfix
    print("TD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "TD",
        {
            'number_of_runs': 1 ,
            'path_prefix': pathfileprefix,
            'number_of_parameters': 2,
            'param_1': 'alpha',
            'param_2': 'lmbda',
            'compare': 'lmbda'
        }
    )

    pathfileprefix = path+"utd/"+postfix
    print("UTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "UTD",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':3,
            'param_1':'eta',
            'param_2':'initd',
            'param_3':'lmbda',
            'compare': 'lmbda'
        }
    )

    pathfileprefix = path+"totd/"+postfix
    print("TOTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "TOTD",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':2,
            'param_1':'alpha',
            'param_2':'lmbda',
            'compare': 'lmbda'
        }
    )

    pathfileprefix = path+"utotd/"+postfix
    print("UTOTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "UTOTD",
        {
            'number_of_runs':1,             # the number of runs in a file
            'path_prefix':pathfileprefix,   # the prefix we use to find the data
            'number_of_parameters':3,       # the number of parameters we compare over
            'param_1':'eta',                # all the parameters
            'param_2':'initd',
            'param_3':'lmbda',
            'compare': 'lmbda'             # the parameter we wish to compare agains
        }
    )

    pathfileprefix = path+"tdr/"+postfix
    print("TDR")
    plot_performance_vs_lambda(
        pathfileprefix,
        "TDR",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':2,
            'param_1':'alpha',
            'param_2':'lmbda',
            'compare': 'lmbda'
        }
    )

    pathfileprefix = path+"utdr/"+postfix
    print("UTDR")
    plot_performance_vs_lambda(
        pathfileprefix,
        "UTDR",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':3,
            'param_1':'eta',
            'param_2':'initd',
            'param_3':'lmbda',
            'compare': 'lmbda'
        }
    )

    ppl.ylim([.0, 0.8])
    # ppl.yscale('log')
    ppl.legend()


if __name__ == '__main__':
    main()
