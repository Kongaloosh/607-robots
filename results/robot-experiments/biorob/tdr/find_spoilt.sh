#!/bin/bash

echo '#!/bin/bash'
alg=tdr
for sub in s1 s2 s3 s4
do
    for act in a1 a2 a3 na1 na2 na3
    do
	for config in {0..103}
	do
	    if [ ! -s 'experiment_'$sub'_'$act'_'$config'.dat' ]
		then
		echo qsub pbs/$alg-$sub-$act-$config.pbs
	    fi
	done
    done
done

