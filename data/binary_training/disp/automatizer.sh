#!/bin/bash

fold=9
stop=0

while [[ stop -eq 0 ]]; do
	cd ../../../../
	bash execute_all_faults.sh
	cd ./data/binary_training/aggr/1.\ raw_folds
	mv ../*.csv ./aggr_fold_${fold}/
	python aa.py
	result=$?
	if [[ $result -eq 0 ]]; then
		stop=1
	fi
done