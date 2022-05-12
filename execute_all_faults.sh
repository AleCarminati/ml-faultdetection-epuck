#!/bin/bash
cd build
make
cd ..

#Declare array
FaultArray=("pmin" "pmax" "prnd" "rofs" "lact" "ract" "bact")

for fault in ${FaultArray[*]};do
	sed -i "s/fault_type=\"[a-z]*\"/fault_type=\"$fault\"/" ./experiments/fault_detection.argos
	argos3 -c experiments/fault_detection.argos
done
