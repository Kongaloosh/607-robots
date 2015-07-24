#!/bin/bash
for s in s1                             # for all the subjects
do
for a in a1 na1                         # for all the sessions in a subject's trial
do
for alg in td totd tdr autotd           # for all the algorithms we want to test
do
echo '#!/bin/bash
#PBS -S /bin/bash
#PBS -M kearney@ualberta.ca
#PBS -m bea
#PBS -l walltime=24:00:00,mem=1gb
#PBS
cd $PBS_O_WORKDIR
echo "Current working directory is `pwd`"
module load application/python/2.7.3
time python pysrc/experiments/prostheticexp.py '$s' '$a' prosthetic_experiment '$alg' plotter > '$alg'-'$s'-'$a'.txt' > $alg-$s-$a.pbs
qsub $alg-$s-$a.pbs
done                                    # end algorithms
done                                    # end actions
done                                    # end subjects


