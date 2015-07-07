# Robot experiments on auto-td and usage-td


This project contains robot experiments evaluating auto-td (Degris et al.) and the usage-based step-size adaptation idea (Mahmood & Sutton 2015).

This project can be imported as an Eclipse Pydev project.

In order to run the experiment on the randomly generated MDP with 10 state and generate plot, execute `run-rndmdp-experiments10.sh`.

In order to run the experiment on the randomly generated MDP with 100 state and generate plot, execute `run-rndmdp-experiments100.sh`.

## Profiling

Use the following to profile the random mdp experiment code:

`python -m cProfile pysrc/experiments/rndmdpexp.py 1000 1 results/rndmdp-experiments/state-10-ftype-binary/ td`

## Unit tests

Use the following from the root directory:

`python -m unittest discover --pattern=*.py`

## References

Mahmood, A. R., Sutton R. S. (2015). Off-policy learning based on weighted importance sampling with linear computational complexity. In *Proceedings of the 301st Conference on Uncertainty in Arti- ficial Intelligence* Amsterdam, Netherlands.



