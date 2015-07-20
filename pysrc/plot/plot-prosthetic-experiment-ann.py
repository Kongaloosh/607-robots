import os
import sys
sys.path.insert(0, os.getcwd())
import matplotlib.pyplot as ppl
from pysrc.plot import plot_data_process_prosthetic
import cPickle as pickle

def plotperfvslmbda(pathfileprefix, label, params):
    plotfilename = pathfileprefix+"perfvs"+params[-1]+".plot.pkl"
    if not os.path.isfile(plotfilename):
        sys.argv = params
        plot_data_process_prosthetic.main()
    plotfile = file(plotfilename, "rb")
    data = pickle.load(plotfile)
    # for i in data:
    #     print i
    ppl.plot(data[:, 2])
    # ppl.errorbar(data[:,0], data[:,1], data[:,2], label=label)

def main():
    path = "results/robot-experiments/biorob/"
    postfix = 'sweep_pool_s1_na1'
    if not os.path.exists(path):
        path = "../." + path

    pathfileprefix = path+"td/"+postfix
    plotperfvslmbda(pathfileprefix, "TD", ["", "1", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utd/"+postfix
    plotperfvslmbda(pathfileprefix, "UTD", ["", "1", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"totd/"+postfix
    plotperfvslmbda(pathfileprefix, "TOTD", ["", "1", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utotd/"+postfix
    plotperfvslmbda(pathfileprefix, "UTOTD", ["", "1", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"tdr/"+postfix
    plotperfvslmbda(pathfileprefix, "TDR", ["", "1", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utdr/"+postfix
    plotperfvslmbda(pathfileprefix, "UTDR", ["", "1", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    ppl.ylim([.0, 0.8])
    # ppl.yscale('log')
    ppl.legend()

if __name__ == '__main__':
    main()
    ppl.show()