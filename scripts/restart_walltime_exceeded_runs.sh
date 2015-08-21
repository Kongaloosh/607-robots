#!/bin/bash
helptext="This script finds the runs which exceeded their walltime and re-starts them for a particular algorithm.
Give the algorithm name, such as 'autotd' to re-submit make sure you edit their wall-time first, otherwise they'll
continue to fail due to exceeding walltime."

if [ -z "$1" ]
then
  echo $helptext
  exit 1
fi

grep "walltime" e/$1* | cut -d'.' -f 1 | cut -d'/' -f 2 | while read -r line; 

do echo "pbs/$line.pbs";
qsub "pbs/$line.pbs";

done

