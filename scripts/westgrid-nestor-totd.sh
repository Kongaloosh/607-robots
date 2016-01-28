#!/bin/bash
#
#   This script goes over all of the algorithms and all of the values using a collection of configuration files
# to organize a sweep to analyse performance at a variety of parameter values. The script is dependant on the
# algorithms specified and requires the data files provided by Ann Edwards to run properly.
#
#   For ease of reading we first generate a set of bash scripts to be run on westgrid, then we call the first of
# these scripts. Automatically, each script will call the next before finishing. This is setup such that we
# baby-sit the sweep
#
#   Very recursion. Much efficency.
#   ░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░
#   ░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░
#   ░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░
#   ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░
#   ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░
#   ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░
#   ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░
#   ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░
#   ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░
#   ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░
#   ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░
#   ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌
#   ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░
#   ░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░
#   ░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░
#   ░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░
#   ░░░░▀▄▒▒▒▒▒▒▒▒▒▒▄▄▄▀▒▒▒▒▄▀░░░
#   ░░░░░░▀▄▄▄▄▄▄▀▀▀▒▒▒▒▒▄▄▀░░░░░
#   ░░░░░░░░░▒▒▒▒▒▒▒▒▒▒▀▀░░░░░░░░


# ====================================================================================================================
#
#                                          CREATE THE SCRIPTS TO RUN
#
# ====================================================================================================================



for alg in td tdr totd autotd               # for all algorithms
do

# while there are still config files available for a process
# make a config for these parameters

echo '
#!/bin/bash
#PBS -S /bin/bash
#PBS -l walltime=12:00:00
#PBS -o o/'$alg'.out
#PBS -e e/'$alg'.err

cd $PBS_O_WORKDIR

echo "Current working directory is `pwd`"
module load application/python/2.7.3
module load python/2.7.2
module load python

for s in 1 2 3 4                            # for all subjects
do
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do

echo pysrc/experiments/prostheticexp.py s$s $a$aval totd '$alg' honors-pos-2016-01-16 -1
time python pysrc/experiments/prostheticexp.py s$s $a$aval totd '$alg' honors-pos-2016-01-16 -1 > txt/'$alg'.txt

done                            # end actions vals
done                            # end actions
done                            # end subjects

echo done
' > pbs/$alg.pbs                           # make a script to run as a process on westgrid
echo 'pbs/'$alg'.pbs'
done                            # end algorithms

# ====================================================================================================================
#
#                                          RUN ALL THE SCRIPTS WE'VE CREATED
#
# ====================================================================================================================

for alg in td tdr totd autotd               # for all algorithms
do
    qsub pbs/$alg.pbs               # make a script to run as a process on westgrid
    echo pbs/$alg.pbs
done
