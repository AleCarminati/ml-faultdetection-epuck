#!/bin/bash
cd build
make
cd ..
if [[ $1 = "m" ]]; then
	argos3 -c experiments/fault_detection_multifault.argos
else
	argos3 -c experiments/fault_detection.argos
fi
