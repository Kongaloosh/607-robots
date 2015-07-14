import os
import sys
os.chdir(os.path.dirname(os.getcwd()))
import matplotlib.pyplot as ppl
from pysrc.plot import plotdataprocess
import cPickle as pickle

def plotperfvslmbda(pathfileprefix, label, params):
    plotfilename = pathfileprefix+"perfvs"+params[-1]+".plot.pkl"
    if not os.path.isfile(plotfilename):
        sys.argv = params
        plotdataprocess.main()
    plotfile = file(plotfilename, "rb")
    data = pickle.load(plotfile)
    ppl.errorbar(data[:,0], data[:,1], data[:,2], label=label)

def main():
    path = "./results/robot-experiments/anns_experiment/"
    if not os.path.exists(path):
        path = "../." + path

    pathfileprefix = path+"td/s1_a1"
    plotperfvslmbda(pathfileprefix, "TD", ["", "50", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utd/s1_a1"
    plotperfvslmbda(pathfileprefix, "UTD", ["", "50", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"totd/s1_a1"
    plotperfvslmbda(pathfileprefix, "TOTD", ["", "50", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utotd/s1_a1"
    plotperfvslmbda(pathfileprefix, "UTOTD", ["", "50", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"tdr/s1_a1"
    plotperfvslmbda(pathfileprefix, "TDR", ["", "50", pathfileprefix, "2", "alpha", "lmbda", "1", "lmbda"])
    pathfileprefix = path+"utdr/s1_a1"
    plotperfvslmbda(pathfileprefix, "UTDR", ["", "50", pathfileprefix, "3", "eta", "initd", "lmbda", "1", "lmbda"])
    ppl.ylim([.0, 0.8])
    #ppl.yscale('log')
    #ppl.legend()

if __name__ == '__main__':
    main()
    ppl.show()