#!/bin/bash
# ====================================================================================================================
#
#                                          RUN ALL THE SCRIPTS WE'VE CREATED
#
# ====================================================================================================================

for alg in autotd               # for all algorithms
do
for s in 4                            # for all subjects
do
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do

var=0

while  [ -f results/robot-experiments/prosthetic_experiment/$alg/configalg_$var.pkl ]; do
    qsub pbs/$alg-s$s-$a$aval-$var.pbs               # make a script to run as a process on westgrid
    echo pbs/$alg-s$s-$a$aval-$var.pbs
((var++))                                           # increment the config number to move to the next file

done                                                # end test if file exists while
done                                                # end actions
done                                                # end algorithms
done
done
