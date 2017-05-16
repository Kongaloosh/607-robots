import os
from scipy.stats.distributions import planck_gen
import sys
sys.path.insert(0, os.getcwd())
from matplotlib import pyplot
import cPickle as pickle
import numpy

# all of the combinations for file labels
subjects = ['s{i}'.format(i=i) for i in range(1, 5)]
actions = numpy.concatenate((['a{i}'.format(i=i) for i in range(1, 4)], ['na{i}'.format(i=i) for i in range(1, 4)]))


def plot_performance_vs_lambda(parameters):
    plot_file_name = parameters['path_prefix'] + "ABS_error_data.pkl" # The file we store results in
    print(plot_file_name)
    if not os.path.isfile(plot_file_name):                                      # if we haven't made a plot file yet
        parameters['data'] = loaddata(parameters['path_prefix'])
        plot_data_process_prosthetic(parameters)                                # collect the data for a plot
        # data = loaddata(parameters['path_prefix'])
        # print(find_best_alpha_lambda(data))
        # p nlot_all_alpha(data)
    else:
        parameters['data'] = pickle.load(open(plot_file_name, 'r'))             # open our pickled results file
        data = parameters['data']
        # print(find_best_alpha_lambda(data))
        # print(find_best_alpha(data, lmbda='0'))
        # plot_all_alpha(data)
        plot_data_process_prosthetic(parameters)


def plot_data_process_prosthetic(parameters):
    """
    We construct a table against a particular parameter
    :param parameters: a dictionary as seen in main(); a multi-dimensional dictionary containing the STD and AVG MSE
    by lambda-alpha combination for a specific algorithm.
    """
    data = parameters['data']                                  # we load all the runs from a trial

    plot_points = find_best_parameters(data)

    pyplot.errorbar(
        [x for (x, y, z) in plot_points],
        [y for (x, y, z) in plot_points],
        yerr=[z for (x, y, z) in plot_points],
        marker='o',
        label=parameters['label']
    )
    pyplot.legend(bbox_to_anchor=(1.11, 1))


def plot_all_alpha(data):
    print(data[data.keys()[0]].keys())
    all_alphas = sorted(data[data.keys()[0]].keys(), key=lambda vals: float(vals))        # the values we wish to vary over
    print(all_alphas)
    for key_alpha in all_alphas:                                         # for all alphas
        if float(key_alpha) < 2:
            print(key_alpha)
            current_points = []                                              # empty array for points
            for key_lambda in data.keys():                                   # lambda

                                                                 # we create a list of tuples with (lambda, ERR, STD)
                current_points.append((
                    float(key_lambda),
                    data[key_lambda][key_alpha]['error'],
                    data[key_lambda][key_alpha]['std'],
                ))                                                                  # append lambda, MSE, STD

            current_points = sorted(current_points, key=lambda vals: vals[0])       # sort data by lambda

            # for (i,err,std) in current_points:
                # print("lambda {0}".format(i))
                # print("alpha {0}".format(key_alpha))
                # print("err {0}".format(err))


            pyplot.figure(0, figsize=(15, 7))

            pyplot.errorbar(
                [lambd for (lambd, x, y) in current_points],
                [error for (w, error, y) in current_points],
                # yerr=[std for (w, x, std) in current_points],
                marker='o',
                label="Alpha {0}".format(key_alpha),
                linewidth=2
            )                                                                       # plot for current alpha

        ax = pyplot.gca()
        ax.get_xaxis().get_major_formatter().set_scientific(False)
        ax.get_xaxis().get_major_formatter().set_useOffset(False)
        ax.set_ylim([0,1000000])
        ax.set_xlim([-0.01,1.1])



        pyplot.ylabel("Mean Absolute Error ", fontsize=18)
        pyplot.xlabel("Values of Lambda", fontsize=18)
        pyplot.title("Performance Of AUTOTD at all Alpha Lambda Combinations", fontsize=24, fontweight='bold', y=1.01)
        pyplot.legend(bbox_to_anchor=(1.11, 1))
        pyplot.savefig("figs/All_Alpha_s_AUTOTD.pdf")
        pyplot.savefig("figs/All_Alpha_s_AUTOTD.png")


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
    print('best lambda-alpha combination: ', key_lambda, best)
    plot_points = sorted(plot_points, key=lambda vals: vals[0])
    print("\n points \n")
    print(plot_points[0])
    return plot_points


def find_best_alpha_lambda(data):
    comparison_values = data.keys()                                             # the values we wish to vary over
    best = None                                                                 # the location of the current best
    best_error = numpy.infty                                                    # the error of the current best
    for key_lambda in comparison_values:                                        # over all of our runs
        for key_alpha in data[key_lambda].keys():
            avg_error = data[key_lambda][key_alpha]['error']
            if avg_error < best_error:
                best_error = avg_error
                best = (key_alpha,key_lambda, best_error)
    return best


def find_best_alpha(data, lmbda):
    best = None                                                                 # the location of the current best
    best_error = numpy.infty                                                    # the error of the current best
    hate = []
    for key_alpha in data[lmbda].keys():
        avg_error = data[lmbda][key_alpha]['error']
        # print(lmbda, key_alpha, data[lmbda][key_alpha]['error'])
        hate.append(avg_error)
        if avg_error < best_error:
            best_error = avg_error
            best = (key_alpha, lmbda, best_error)
    print(numpy.array(hate).mean() ,numpy.array(hate).std())
    return best


def plot_learning_curve(parameters):
    data = load_data_one_parameter(parameters['path_prefix'])
    pyplot.plot(data['error'],marker='o', lw=0, ms=1)


def load_data_one_parameter(path):
    """for when we only have one parameter combination"""
    data = []
    for a in actions:                                                       # for all actions
        print(a)
        for s in subjects:                                                  # for all subjects
            print(s)
            filepathname = path + "_{s}_{a}.dat" .format(s=s, a=a)          # point to data file
            try:
                f = open(filepathname, 'rb')                                # load the file results are stored in
                try:                                                        # we catch for the end of the file
                    while True:                                             # until we break out of the reading loop
                        d = pickle.load(f)                                  # get a run
                        lmbda = str(d['lmbda'])                             # get the current lambda's error
                        try:
                            alpha = str(d['alpha'])
                        except KeyError:
                            alpha = str(d['initalpha'])

                        trial_error = d['error']                  # /sum(d['return'])    # get normalized error
                        data.append(trial_error)
                except EOFError:
                    pass
            except IOError:
                print "File doesn't exist " + path + "_{s}_{a}.dat" .format(s=s,a=a)
    truncate_at = min([len(trial) for trial in data])
    runs = [(run[:truncate_at] ** 2) for run in data]                   # mean squared error for all
    data = {}                                                           # dict for SE and STD
    collapsed = [[runs[y][z] for y in range(len(runs))] for z in range(len(runs[0]))] #badSWE
    data['error'] = [ numpy.mean(collapsed[x]) for x in range(len(collapsed))]
    pickle.dump(data, open(path+"_data.pkl",'wb'))
    return data

# def loaddata(path):
#     """
#      Takes in the number of runs---or, files we have---and looks for all the files
#     for any given file, we load all of the results and find.
#
#     Remember, the results are going to be a collection of pickled items, each
#     being one member of a sweep for a particular episode.
#
#     :param path: the prefix to where we want to find result file
#
#     :returns data: a multidimensional dic which contains the AVG MSE and STD for every alpha lambda combination
#     """
#     data = {}
#     for a in actions:                                                       # for all actions
#         print(a)
#         for s in subjects:                                                  # for all subjects
#             print(s)
#             filepathname = path + "_{s}_{a}.dat" .format(s=s,a=a)           # point to data file
#             try:
#                 f = open(filepathname, 'rb')                                # load the file results are stored in
#                 try:                                                        # we catch for the end of the file
#                     i = 0
#                     while True:                                             # until we break out of the reading loop
#                         d = pickle.load(f)                                  # get a run
#                         lmbda = str(d['lmbda'])                             # get the current lambda's error
#                         try:
#                             alpha = str(d['alpha'])
#                         except KeyError:
#                             alpha = str(d['initalpha'])
#
#                         trial_error = d['error']                            # get reported error
#                         # /sum(d['return'])    # get normalized error
#                         try:
#                             data[lmbda][alpha].append(
#                                 trial_error
#                             )                                               # add the abs error
#                         except KeyError:                                    # if this lambda is not in the dict yet
#                             try:
#                                 data[lmbda][alpha] = []                     # make a list for the key
#                             except KeyError:
#                                 data[lmbda] = {}
#                                 data[lmbda][alpha] = []                     # make a list for the key
#
#                             data[lmbda][alpha].append(
#                                 trial_error
#                             )                                               # add the abs error
#                         i += 1
#                 except EOFError:
#                     pass
#             except IOError:
#                 print "File doesn't exist " + path + "_{s}_{a}.dat" .format(s=s,a=a)
#
#     # truncate all of the errors over the max to ensure consistent time-steps:
#     lmbda = data.keys()[0]
#     alpha = data[lmbda].keys()[0]
#
#     # every parameter combination has all sessions,
#     # so we just need to find the minimum for one combo and it will generalize
#     truncate_at = min([len(trial) for trial in data[lmbda][alpha]])
#     print(truncate_at)
#     for key_lambda in data.keys():
#         for key_alpha in data[key_lambda].keys():
#             runs = [numpy.sum(run[:truncate_at] ** 2) for run in data[key_lambda][key_alpha]]   # cumu squared error
#             data[key_lambda][key_alpha] = {}                                                    # dict for MSE and STD
#             data[key_lambda][key_alpha]['error'] = numpy.mean(runs)                             # avg MSE
#             data[key_lambda][key_alpha]['std'] = numpy.std(runs)                                # STD
#     pickle.dump(data, open(path+"_data.pkl",'wb'))
#     return data

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
                f = open(filepathname, 'rb')                            # load the file results are stored in
                try:                                                    # we catch for the end of the file
                    i = 0
                    while True:                                         # until we break out of the reading loop
                        d = pickle.load(f)                              # get a run
                        lmbda = str(d['lmbda'])                         # get the current lambda's error
                        try:
                            alpha = str(d['alpha'])
                        except KeyError:
                            alpha = str(d['initalpha'])

                        # CUMU ABS ERROR
                        trial_error = numpy.sum(numpy.abs(d['error']))
                        print(trial_error)

                        # MSE
                        # trial_error = numpy.mean(numpy.array(d['prediction'][:len(d['return'])]-d['return'])**2)


                        # trial_error = d['error'] / sum(d['return'])       # /sum(d['return'])    # get normalized error
                        # trial_error = d

                        try:
                            data[lmbda][alpha].append(
                                trial_error
                            )                                           # add the abs error
                        except KeyError:                                # if this alpha is not in the dict yet
                            try:
                                data[lmbda][alpha] = []                 # make a list for the key
                            except KeyError:                            # if we don't have the lambda yed
                                data[lmbda] = {}                        # add a dict
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
    # truncate_at = min([len(trial) for trial in data[lmbda][alpha]])
    # print(truncate_at)
    for key_lambda in data.keys():
        for key_alpha in data[key_lambda].keys():
            # runs = [(run[:truncate_at] ** 2).mean() for run in data[key_lambda][key_alpha]] # mean squared error for all

            # runs = [(run[:truncate_at]).mean() for run in data[key_lambda][key_alpha]] # mean abs error for all

            # try:
            #     runs = numpy.mean(data[key_lambda][key_alpha], axis=1) / numpy.mean(data['0'][key_alpha], axis=1)
            # except:
            #     runs = numpy.mean(data[key_lambda][key_alpha], axis=1) / data['0'][key_alpha]['error']

            runs = numpy.array(data[key_lambda][key_alpha])

            # runs = [
            #     (numpy.array(run['prediction'][:truncate_at] - run['return'][:truncate_at])**2).mean()
            #     for
            #     run
            #     in data[key_lambda][key_alpha]
            # ]
            data[key_lambda][key_alpha] = {}                                                # dict for MSE and STD
            data[key_lambda][key_alpha]['error'] = runs.mean()                              # avg MSE
            data[key_lambda][key_alpha]['std'] = numpy.std(runs)                            # STD
            print("error {0} and std {1}".format(data[key_lambda][key_alpha]['error'], data[key_lambda][key_alpha]['std']))
    # pickle.dump(data, open(path + "abs_error_data.pkl", 'wb'))
    pickle.dump(data, open(path + "ABS_error_data.pkl", 'wb'))
    return data


def main():
    """
    Constructs a file comparing the algs across for each alpha
    :returns none:
    """
    path = "results/robot-experiments/totd/"                            # the path to the experiments
    #postfix = 'full_run'                                               # the name of the experiment run
    postfix = 'total_run'                                               # the name of the experiment run

    #pathfileprefix = path + "td/" + postfix
    #print("TD")
    #plot_performance_vs_lambda(
    #    {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label': 'TD'}
    #)

    #pathfileprefix = path + "totd/" + postfix
    #print("TOTD")
    #plot_performance_vs_lambda(
    #   {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label': 'TOTD'}
    #)

    #pathfileprefix = path + "tdr/" + postfix
    #print("TDR")
    #plot_performance_vs_lambda(
    #    {'path_prefix':pathfileprefix, 'compare': 'lmbda', 'label': 'TDR'}
    #)
    #

    #pathfileprefix = path + "autotd/" + postfix
    #plot_performance_vs_lambda(
    #    {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label':'AUTOTD'}
    #)

    pathfileprefix = path + "dasautotdr/" + postfix
    plot_performance_vs_lambda(
        {'path_prefix': pathfileprefix, 'compare': 'lmbda', 'label':'DASAUOTD'}
    )

    pyplot.title("Average Abs Error \n Across All Users and Trials For Hand Position Predictions", fontsize=24, fontweight="bold")
    ylabel = "AVG ABS Error Over All Trials and Subjects"
    pyplot.ylabel(ylabel, fontsize=18)
    pyplot.xlabel("Values of Lambda", fontsize=18)
    pyplot.xlim([-0.1, 1.1])

    pyplot.savefig('figs/everything.png')
    pyplot.show()

if __name__ == '__main__':
    main()
