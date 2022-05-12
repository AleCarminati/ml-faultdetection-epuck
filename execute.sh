#!/bin/bash
cd build
make
cd ..
argos3 -c experiments/fault_detection.argos
