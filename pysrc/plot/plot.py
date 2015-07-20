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
        plot_data_process_prosthetic(parameters, path_prefix, label)
        # plotfile = file(plot_file_name, "rb")
        # data = pickle.load(plotfile)
        # # print(data[:, 2])
        # ppl.plot(data[:, 2])
        # ppl.errorbar(data[:,0], data[:,1], data[:,2], label=label)


def plot_data_process_prosthetic(parameters, path_prefix, label):
    """
    We construct a table against a particular parameter
    :param parameters: a dictionary as seen in main()
    :param path_prefix: where the path goes to
    """
    # the different values we compare performance across
    comparison_val = parameters['compare']                          # the name of the variable we compare across
    data = loaddata(path_prefix)                                    # we load all the runs from a trial
    comparison_values = set([run[comparison_val] for run in data])  # we find the values we wish to vary over

    plot_points = []
    for run_parameter in comparison_values:                                     # over all of our runs
        best = None
        best_error = numpy.infty
        for run in data:                                                        # for each run we perform
            new_error = sum(abs(run['error']))                                  # we determine the absolute error
            if run[comparison_val] == run_parameter and new_error < best_error: # if it's the current best
                best_error = new_error                                          # update our error tracker
                best = run                                                      # update our run tracker
        plot_points.append(best)                                                # append the best
    plot_points = sorted(plot_points, key=lambda k: k[comparison_val])          # sort the points by comparator var

    pyplot.plot(
        [i[comparison_val] for i in plot_points],
        [sum(abs(i['error'])) for i in plot_points],
        marker='o',
        label=label
    )


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

    pathfileprefix = path + "td/" + postfix
    print("TD")
    plot_performance_vs_lambda(
        pathfileprefix, "TD", {'path_prefix': pathfileprefix, 'compare': 'lmbda'}
    )

    pathfileprefix = path + "utd/" + postfix
    print("UTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "UTD",
        {'path_prefix': pathfileprefix, 'compare': 'lmbda'}
    )

    pathfileprefix = path + "totd/" + postfix
    print("TOTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "TOTD", {'path_prefix':pathfileprefix, 'compare': 'lmbda'}
    )

    pathfileprefix = path + "utotd/" + postfix
    print("UTOTD")
    plot_performance_vs_lambda(
        pathfileprefix, "UTOTD", {'path_prefix':pathfileprefix, 'compare': 'lmbda'}
    )

    pathfileprefix = path + "tdr/" + postfix
    print("TDR")
    plot_performance_vs_lambda(
        pathfileprefix, "TDR", {'path_prefix':pathfileprefix, 'compare': 'lmbda'}
    )

    pathfileprefix = path + "utdr/" + postfix
    print("UTDR")
    plot_performance_vs_lambda(
        pathfileprefix, "UTDR", {'path_prefix': pathfileprefix, 'compare': 'lmbda'}
    )

    # ppl.ylim([.0, 0.8])
    # ppl.yscale('log')
    # ppl.legend()
    pyplot.show()

if __name__ == '__main__':
    main()