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

var=0

while  [ -f results/robot-experiments/prosthetic_experiment/$alg/configalg_$var.pkl ]; do
# while there are still config files available for a process
# make a config for these parameters

echo '
#!/bin/bash
#PBS -S /bin/bash
#PBS -M hi@kongaloosh.com                   # e-mail results to this place
#PBS -m a                                   # optional begining end abort, only abort e-mails
#PBS -l walltime=24:00:00,mem=1gb           # give our scripts 24 hours with a gig of ram
#PBS -o o/'$alg'-s'$s'-'$a$aval'-'$var'.out
#PBS -e e/'$alg'-s'$s'-'$a$aval'-'$var'.err
#PBS
cd $PBS_O_WORKDIR

echo "Current working directory is `pwd`"
module load application/python/2.7.3

time python pysrc/experiments/prostheticexp.py s'$s' '$a$aval' prosthetic_experiment '$alg' distributed '$var' > txt/'$alg'-'$s'-'$a'-'$var'.txt

if ['$aval' < '3']                                          # if we are at the last trial...
    qsub pbs/'$alg'-s'$(($s+1))'-'$a$aval'-'$var'.pbs       # move to the next person and the next trial
elif ['$s' < '4']                                           # if we still have trials left
    qsub pbs/'$alg'-s'$s'-'$a$(($aval+1))'-'$var'.pbs       # move to the next trial
fi
' > pbs/$alg-s$s-$a$aval-$var.pbs                           # make a script to run as a process on westgrid

((var++))                       # increment the config number to move to the next file
done                            # end test if file exists while
done                            # end actions vals
done                            # end actions
done                            # end subjects
done                            # end algorithms

# ====================================================================================================================
#
#                                          RUN ALL THE SCRIPTS WE'VE CREATED
#
# ====================================================================================================================

for alg in td tdr totd autotd                       # for all algorithms
do
for a in a na                                       # for all actions in a session
do
s=1
aval=1                                              # the first trial is where we start all our runs
var=0

while  [ -f results/robot-experiments/prosthetic_experiment/$alg/configalg_$var.pkl ]; do
    #qsub pbs/$alg-s$s-$a$aval-$var.pbs               # make a script to run as a process on westgrid
    echo pbs/$alg-s$s-$a$aval-$var.pbs
((var++))                                           # increment the config number to move to the next file

done                                                # end test if file exists while
done                                                # end actions
done                                                # end algorithms


