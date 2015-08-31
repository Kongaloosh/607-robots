#!/bin/bash
alg = $1
experiment = $2

if [[ ! (-n "$alg")]]
then
    echo "We re-run all experiments with empty results files. Argument One: the alg. Argument Two: the experiment to check"
fi

if [[ ! (-n "$experiment")]]
then
    echo "We re-run all experiments with empty results files. Argument One: the alg. Argument Two: the experiment to check"
fi

	find results/robot-experiments/$experiment/$alg -size 0 | rev | cut -d '/' -f 1 | rev | cut -d '.' -f 1 | rev | cut -d '_' -f 1-3 | rev | tr '_' '-'|while read -r line
	do echo "pbs/$alg-$line.pbs"
	qsub "pbs/$alg-$line.pbs"
	done


