#!/usr/bin/env bash

alg=alphabound

#for s in 1 2 3 4                            # for all subjects
#do
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do
var=0
echo $s $a $aval $var

while  [ -f results/robot-experiments/totd/$alg/configalg_$var.pkl ]; do
    python pysrc/experiments/prostheticexp.py s$1 $a$aval totd $alg experiment $var
((var++))                       # increment the config number to move to the next file
done                            # end test if file exists while
done                            # end actions vals
done                            # end actions
#done
