import os
import sys
sys.path.insert(0, os.getcwd())
from matplotlib import pyplot
import cPickle as pickle
import numpy

subjects = ['s{i}'.format(i=i) for i in range(1, 5)]
# actions = numpy.concatenate((['a{i}'.format(i=i) for i in range(1, 4)], ['na{i}'.format(i=i) for i in range(1, 4)]))
actions = ['a{i}'.format(i=i) for i in range(1, 4)]


def plot_performance_vs_lambda(parameters):
    plot_file_name = parameters['path_prefix'] + "perf_vs_lambda" + ".plot.pkl" # The file we store results in
    if not os.path.isfile(plot_file_name):                                      # if we haven't made a plot file yet
        plot_data_process_prosthetic(parameters)                                # collect the data for a plot


# def plot_data_process_prosthetic(parameters):
#     """
#     We construct a table against a particular parameter
#     :param parameters: a dictionary as seen in main()
#     :param path_prefix: where the path goes to
#     """
#     comparison_val = parameters['compare']                                      # the variable we compare across
#     data = loaddata(parameters['path_prefix'])                                  # we load all the runs from a trial
#     comparison_values = set([run[comparison_val] for run in data])              # the values we wish to vary over
#
#     plot_points = []
#     for run_parameter in comparison_values:                                     # over all of our runs
#         best = None                                                             # the location of the current best
#         best_error = numpy.infty                                                # the error of the current best
#         for run in data:                                                        # for each run we perform
#             # get the avg error
#             new_error = run                                                     # we determine the absolute error
#             if run[comparison_val] == run_parameter and new_error < best_error: # if it's the current best
#                 best_error = new_error                                          # update our error tracker
#                 best = run                                                      # update our run tracker
#         plot_points.append(best)                                                # append the best
#     plot_points = sorted(plot_points, key=lambda k: k[comparison_val])          # sort the points by comparator var
#
#     pyplot.plot(
#         [i[comparison_val] for i in plot_points],
#         [sum(abs(i['error'])) for i in plot_points],
#         marker='o',
#         label=parameters['label']
#     )
#
#     pyplot.legend()


def plot_data_process_prosthetic(parameters):
    """
    We construct a table against a particular parameter
    :param parameters: a dictionary as seen in main()
    :param path_prefix: where the path goes to
    """
    comparison_val = parameters['compare']                                      # the variable we compare across
    data = loaddata(parameters['path_prefix'])                                  # we load all the runs from a trial
    comparison_values = set([run[comparison_val] for run in data])              # the values we wish to vary over

    plot_points = []
    for run_parameter in comparison_values:                                     # over all of our runs
        best = None                                                             # the location of the current best
        best_error = numpy.infty                                                # the error of the current best
        for run in data:                                                        # for each run we perform
            # get the avg error
            new_error = run                                                     # we determine the absolute error
            if run[comparison_val] == run_parameter and new_error < best_error: # if it's the current best
                best_error = new_error                                          # update our error tracker
                best = run                                                      # update our run tracker
        plot_points.append(best)                                                # append the best
    plot_points = sorted(plot_points, key=lambda k: k[comparison_val])          # sort the points by comparator var

    pyplot.plot(
        [i[comparison_val] for i in plot_points],
        [sum(abs(i['error'])) for i in plot_points],
        marker='o',
        label=parameters['label']
    )

    pyplot.legend()


# def loaddata(path, nruns = 1):
#     """
#      Takes in the number of runs---or, files we have---and looks for all the files
#     for any given file, we load all of the results and find.
#
#     Remember, the results are going to be a collection of pickled items, each
#     being one member of a sweep for a particular episode.
#
#     :param nruns: number of runs
#     :param path: the prefix to where we want to find result files
#     """
#     data = []
#     element = None
#     pickles = []
#     for a in actions:
#         for s in subjects:
#             filepathname = path + "_{s}_{a}.dat" .format(s=s,a=a)       # point to data file
#             f = open(filepathname, 'rb')                                # load the file results are stored in
#             pickles.append(f)                                           # curate a list of all vals
#     try:                                                                # While we still have results in a given file
#         while True:                                                     # load and append the data to our array
#             avg = []                                                    # where we store our current avg
#             for p in pickles:                                           # for all relevant files
#                 try:
#                     d = pickle.load(p)                                  # load and append a run
#                     error = abs(sum(abs(d['error'])))                   # find the error for this config
#                     avg.append(error)                                   # append the error
#                     element = d                                         # set the current element
#                 except:
#                     pass
#
#             if len(avg) > 0:                                            # if we've got an avg
#                 d = element                                             # store the config of the run we analysed
#                 d['error_avg'] = (sum(avg) / float(len((avg))))         # calculate the avg error over all files
#                 data.append(d)                                          # append the value
#     except EOFError:                                                    # catch the end of the file to move on
#         print 'End of file reached'
#     return data                                                         # pass all runs back and find minimum avg


def loaddata(path, nruns = 1):
    """
     Takes in the number of runs---or, files we have---and looks for all the files
    for any given file, we load all of the results and find.

    Remember, the results are going to be a collection of pickled items, each
    being one member of a sweep for a particular episode.

    :param nruns: number of runs
    :param path: the prefix to where we want to find result files
    """
    data = {}
    for a in actions:                                                   # for all actions
        print(a)
        for s in subjects:                                              # for all subjects
            print(s)
            filepathname = path + "_{s}_{a}.dat" .format(s=s,a=a)       # point to data file
            f = open(filepathname, 'rb')                                # load the file results are stored in
            try:                                                        # we catch for the end of the file
                while True:                                             # until we break out of the reading loop
                    d = pickle.load(f)                                  # get a run
                    lmbda = str(d['lmbda'])                             # get the current lambda's error
                    try:
                        data[lmbda].append(sum(abs(d['error'])))        # add the abs error
                    except KeyError:                                    # if this lambda is not in the dict yet
                        data[lmbda] = []                                # make a list for the key
                        data[lmbda].append(sum(abs(d['error'])))        # add the abs error
            except EOFError:
                pass
    for lmbda in data.keys():                                           # for all the configs
        data[lmbda] = sum(data[lmbda]) / float(len(data[lmbda]))        # get the avg error
    print(data)
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

    # pathfileprefix = path + "utdr/" + postfix
    # plot_performance_vs_lambda(
    #     {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label':'UTDR'}
    # )

    pyplot.show()

if __name__ == '__main__':
    main()