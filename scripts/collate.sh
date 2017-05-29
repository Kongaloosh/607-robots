#!/bin/bash

# ====================================================================================================================
#
#                                          CREATE THE SCRIPTS TO RUN
#
# ====================================================================================================================


for alg in alphabound
do
echo $alg
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do

python pysrc/plot/weird_collator.py $alg s$1 $a$aval
echo $a$aval

done
done
done
