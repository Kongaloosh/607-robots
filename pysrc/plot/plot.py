import os
import sys
sys.path.insert(0, os.getcwd())
from matplotlib import pyplot
import cPickle as pickle
import numpy

# all of the combinations for file labels
subjects = ['s{i}'.format(i=i) for i in range(1, 5)]
actions = numpy.concatenate((['a{i}'.format(i=i) for i in range(1, 4)], ['na{i}'.format(i=i) for i in range(1, 4)]))


def plot_performance_vs_lambda(parameters):
    plot_file_name = parameters['path_prefix'] + "perf_vs_lambda" + ".plot.pkl" # The file we store results in
    if not os.path.isfile(plot_file_name):                                      # if we haven't made a plot file yet
        plot_data_process_prosthetic(parameters)                                # collect the data for a plot
    else:
        open(plot_file_name)
        data = pickle.load(plot_file_name)
        plot_data_process_prosthetic(data)


def plot_data_process_prosthetic(parameters):
    """
    We construct a table against a particular parameter
    :param parameters: a dictionary as seen in main(); a multi-dimensional dictionary containing the STD and AVG MSE
    by lambda-alpha combination for a specific algorithm.
    """
    data = loaddata(parameters['path_prefix'])                                  # we load all the runs from a trial

    plot_points = find_best_parameters(data)

    pyplot.errorbar(
        [x for (x, y, z) in plot_points],
        [y for (x, y, z) in plot_points],
        yerr=[z for (x, y, z) in plot_points],
        marker='o',
        label=parameters['label']
    )
    pyplot.legend()


def find_best_parameters(data):
    plot_points = []
    comparison_values = data.keys()                                             # the values we wish to vary over
    for key_lambda in comparison_values:                                        # over all of our runs
        best = None                                                             # the location of the current best
        best_error = numpy.infty                                                # the error of the current best
        for key_alpha in data[key_lambda].keys():
            avg_error = data[key_lambda][key_alpha]['error']
            if avg_error < best_error:
                best_error = avg_error
                best = key_alpha
        plot_points.append((float(key_lambda), data[key_lambda][best]['error'], data[key_lambda][best]['std']))
    plot_points = sorted(plot_points, key=lambda vals: vals[0])
    return plot_points

def loaddata(path):
    """
     Takes in the number of runs---or, files we have---and looks for all the files
    for any given file, we load all of the results and find.

    Remember, the results are going to be a collection of pickled items, each
    being one member of a sweep for a particular episode.

    :param path: the prefix to where we want to find result file

    :returns data: a multidimensional dic which contains the AVG MSE and STD for every alpha lambda combination
    """
    data = {}
    for a in actions:                                                   # for all actions
        print(a)
        for s in subjects:                                              # for all subjects
            print(s)
            filepathname = path + "_{s}_{a}.dat" .format(s=s,a=a)       # point to data file
            try:
                f = open(filepathname, 'rb')                                # load the file results are stored in
                try:                                                        # we catch for the end of the file
                    i = 0
                    while True:                                             # until we break out of the reading loop
                        d = pickle.load(f)                                  # get a run
                        lmbda = str(d['lmbda'])                             # get the current lambda's error
                        try:
                            alpha = str(d['alpha'])
                        except KeyError:
                            alpha = str(d['initalpha'])

                        trial_error = d['error']                  # /sum(d['return'])    # get normalized error
                        try:
                            data[lmbda][alpha].append(
                                trial_error
                            )                                           # add the abs error
                        except KeyError:                                         # if this lambda is not in the dict yet
                            try:
                                data[lmbda][alpha] = []                 # make a list for the key
                            except KeyError:
                                data[lmbda] = {}
                                data[lmbda][alpha] = []                 # make a list for the key

                            data[lmbda][alpha].append(
                                trial_error
                            )                                           # add the abs error
                        i += 1
                except EOFError:
                    pass
            except IOError:
                print "File doesn't exist " + path + "_{s}_{a}.dat" .format(s=s,a=a)

    # truncate all of the errors over the max to ensure consistent time-steps:
    lmbda = data.keys()[0]
    alpha = data[lmbda].keys()[0]

    # every parameter combination has all sessions,
    # so we just need to find the minimum for one combo and it will generalize
    truncate_at = min([len(trial) for trial in data[lmbda][alpha]])

    for key_lambda in data.keys():
        for key_alpha in data[key_lambda].keys():
            runs = [(run[:truncate_at] ** 2).mean() for run in data[key_lambda][key_alpha]] # mean squared error for all
            data[key_lambda][key_alpha] = {}                                                # dict for MSE and STD
            data[key_lambda][key_alpha]['error'] = sum(runs) / len(runs)                    # avg MSE
            data[key_lambda][key_alpha]['std'] = numpy.std(runs)                            # STD
    return data


def main():
    """
    Constructs a file comparing the algs across for each alpha
    :returns none:
    """
    path = "results/robot-experiments/prosthetic_experiment/"         # the path to the experiments
    postfix = 'total_run'                                             # the name of the experiment run

    pathfileprefix = path + "td/" + postfix
    print("TD")
    plot_performance_vs_lambda(
        {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label': 'TD'}
    )

    # pathfileprefix = path + "utd/" + postfix
    # print("UTD")
    # plot_performance_vs_lambda(
    #     {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label': 'UTD'}
    # )

    pathfileprefix = path + "totd/" + postfix
    print("TOTD")
    plot_performance_vs_lambda(
       {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label': 'TOTD'}
    )

    # pathfileprefix = path + "utotd/" + postfix
    # print("UTOTD")
    # plot_performance_vs_lambda(
    #     {'path_prefix':pathfileprefix, 'compare': 'lmbda', 'label': 'UTOTD'}
    # )

    pathfileprefix = path + "tdr/" + postfix
    print("TDR")
    plot_performance_vs_lambda(
       {'path_prefix':pathfileprefix, 'compare': 'lmbda', 'label': 'TDR'}
    )

    pathfileprefix = path + "autotd/" + postfix
    plot_performance_vs_lambda(
        {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label':'AUTOTD'}
    )

    pyplot.show()

if __name__ == '__main__':
    main()
