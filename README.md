# Robot experiments on auto-td and usage-td


This project contains robot experiments evaluating auto-td (Degris et al.) and the usage-based step-size adaptation idea (Mahmood & Sutton 2015).

This project can be imported as an Eclipse Pydev project.

In order to run the experiment on the randomly generated MDP with 10 state and generate plot, execute `run-rndmdp-experiments10.sh`.

In order to run the experiment on the randomly generated MDP with 100 state and generate plot, execute `run-rndmdp-experiments100.sh`.

## Running Experiments Using Prosthetic Data

The experiments can be broken into three pieces: the `experiment`, the `problem`, and the `learning algorithm`.

#### The Experiment
The Experiment handles the data, the algorithm's configuration files, and the problem's configuration files. 

For instance, the following line constructs a prosthetic experiment on subject one (s1), adaptive trial one (a1), using the biorob problem (biorob), using algorithm TOTD (totd), naming the results file_name, using configuration file 1.

`python pysrc/experiments/prosthetic_experiment.py s1 a1 biorob totd file_name 1`

Given these, it will pass the observations to the problem, which will in turn generate the parameters for the next step in the learning algorithm. These are all 

Finally, the results are stored in `pysrc/robot-experiments/<problem>/<algorithm>/<file_name>.dat`

#### The Problem
The Problem---given a set of observations---defines phi, gamma, gamma next, and reward.

#### Directory Tree
```
usage-td-experiments-robot/
  |
  |---- pysrc/
  |       |
  |       |--- experiments/                       
  |       |         |
  |       |         |---- prostheticexp.py        (bulids the problem and runs it)
  |       |
  |       |--- problems/                          (defines phi, r, and other values to be passed to td, totd,.etc)
  |                 |
  |                 |---- prosthetic_problem.py   (collection of different problems for running with prosthetic data)
  |
  |
  |---- results/
  |         |
  |         |--- prosthetic-data/                 (data from Ann's adaptive and non-adaptive prosthetic trials)
  |         |
  |         |--- robot-experiments/     
  |                  |              
  |                  |
  |                  |-- biorob                   (folders with each problem's configuration)
  |                  |
  |                  |-- prosthetic_experiment    (folders with each problem's configuration)
```

___

#### Exploring Results

For ease, we collect all the different run configurations into one file before we plot. To speed the collation up, we crimp the file down to only the values we need: error, lambda, and alpha. 

so, an example setup for plotting all experiments would be 

1. `python pysrc/utilities/file_crimper.py autotd,totd,td,tdr`
2. `python pysrc/plot/experiment_collator.py autotd,totd,td,tdr`
3. `python pysrc/plot/plot.py` 

## Profiling

Use the following to profile the random mdp experiment code:

`python -m cProfile pysrc/experiments/rndmdpexp.py 1000 1 results/rndmdp-experiments/state-10-ftype-binary/ td`
e
## Unit tests

Use the following from the root directory:

`python -m unittest discover --pattern=*.py`

## References

Mahmood, A. R., Sutton R. S. (2015). Off-policy learning based on weighted importance sampling with linear computational complexity. In *Proceedings of the 301st Conference on Uncertainty in Arti- ficial Intelligence* Amsterdam, Netherlands.



