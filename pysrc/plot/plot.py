import os
import sys
sys.path.insert(0, os.getcwd())
import matplotlib.pyplot as ppl
from pysrc.plot import plot_data_process_prosthetic
import cPickle as pickle


def plot_performance_vs_lambda(path_prefix, label, parameters):
    plot_file_name = path_prefix + "perfvs" + parameters[-1] + ".plot.pkl"
    if not os.path.isfile(plot_file_name):  # if we haven't made a plot file yet
        plot_data_process_prosthetic(parameters)
    plotfile = file(plot_file_name, "rb")
    data = pickle.load(plotfile)
    # print(data[:, 2])
    ppl.plot(data[:, 2])
    ppl.errorbar(data[:,0], data[:,1], data[:,2], label=label)


def plot_data_process_prosthetic(parameters):


def main():
    path = "results/robot-experiments/biorob/"
    postfix = 'sweep_pool_2_s1_na1'
    if not os.path.exists(path):
        path = "../." + path

    pathfileprefix = path+"td/"+postfix
    print("TD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "TD",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':2,
            'param_1':'alpha',
            'param_2':'lambda'
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
            'param_3':'lambda'
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
            'param_2':'lambda'
        }
    )

    pathfileprefix = path+"utotd/"+postfix
    print("UTOTD")
    plot_performance_vs_lambda(
        pathfileprefix,
        "UTOTD",
        {
            'number_of_runs':1 ,
            'path_prefix':pathfileprefix,
            'number_of_parameters':3,
            'param_1':'eta',
            'param_2':'initd',
            'param_3':'lambda'
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
            'param_2':'lambda'
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
            'param_3':'lambda'
        }
    )

    ppl.ylim([.0, 0.8])
    # ppl.yscale('log')
    ppl.legend()

    if __name__ == '__main__':
        main()
    ppl.show()