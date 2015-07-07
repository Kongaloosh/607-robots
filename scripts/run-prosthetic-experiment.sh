#!/bin/bash

for s in s1 s2 s3 s4
do

for a in a1 a2 a3 na1 na2 na3
do

for alg in tdr utdr td totd utd utotd
do

# time python pysrc/experiments/prostheticexp.py $s $a anns_experiment $alg &
time python pysrc/experiments/prostheticexp.py $s $a anns_experiment $alg

done

done

done

echo "Invoking matplotlib plot for the experiment ..."
python ./pysrc/plot/plotrndmdpexp10.py

