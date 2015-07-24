#!/bin/bash
for s in s1                     # for all subjects
do
for a in a1                 # for all actions in a session
do
for alg in td  # for all algorithms
do
var=0
while  [ -f results/robot-experiments/prosthetic_experiment/$alg/configalg_$var.pkl ]; do
# while there are still config files available for a process
echo '
#!/bin/bash
#PBS -S /bin/bash
#PBS -M hi@kongaloosh.com
#PBS -m bea
#PBS -l walltime=00:40:00,mem=1gb
#PBS
cd $PBS_O_WORKDIR
echo "Current working directory is `pwd`"
module load application/python/2.7.3
time python pysrc/experiments/prostheticexp.py '$s' '$a' prosthetic_experiment '$alg' distributed '$var' > txt/'$alg'-'$s'-'$a'-'$var'.txt
' > pbs/$alg-$s-$a-$var.pbs              # make a script to run as a process on westgrid
qsub pbs/$alg-$s-$a-$var.pbs             # add the process to the queue on westgrid
((var++))                       # increment the config number to move to the next file
done                            # end test if file exists while
done                            # end algorithms
done                            # end actions
done                            # end subjects