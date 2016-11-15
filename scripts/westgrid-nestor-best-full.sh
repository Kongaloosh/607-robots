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
for s in 1 2 3 4                            # for all subjects
do
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do
# while there are still config files available for a process
# make a config for these parameters

echo '
#!/bin/bash
#PBS -S /bin/bash
#PBS -l walltime=24:00:00
#PBS -o o/'$alg'-s'$s'-'$a$aval'-'$var'.out
#PBS -e e/'$alg'-s'$s'-'$a$aval'-'$var'.err

cd $PBS_O_WORKDIR

echo "Current working directory is `pwd`"
#module load application/python/2.7.3
module load python/2.7.2

time python pysrc/experiments/prostheticexp.py s'$s' '$a$aval' prosthetic_experiment '$alg' full_run -1 > txt/'$alg'-s'$s'-'$a$aval'.txt
echo done
' > pbs/$alg-s$s-$a$aval.pbs                           # make a script to run as a process on westgrid
done                            # end actions vals
done                            # end actions
done                            # end subjects
done                            # end algorithms

# ====================================================================================================================
#
#                                          RUN ALL THE SCRIPTS WE'VE CREATED
#
# ====================================================================================================================

for alg in td tdr totd			# for all algorithms
do
for s in 1 2 3 4                            # for all subjects
do
for a in a na                               # for all actions in a session
do
for aval in 1 2 3                           # for the number of trials per person
do
    qsub pbs/$alg-s$s-$a$aval.pbs               # make a script to run as a process on westgrid
    echo pbs/$alg-s$s-$a$aval.pbs                                                # end test if file exists while
done                                                # end actions
done                                                # end algorithms
done
done
