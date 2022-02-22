# Machine Learning-based fault detection in a e-puck swarm

This repository contains the code to run a machine learning-based fault detection algorithm in a e-puck swarm. 

It is part of my master's thesis work in Politecnico di Milano to obtain the Masters of Science in Computer Science and Engineering and in Mathematical Engineering.	  

## Installation

The code is tested on the version 3.0.0-beta56 of the [ARGoS simulator](https://www.argos-sim.info/). 

To download this folder, use the following command-line instruction:

```bash
git clone --recurse-submodule git@github.com:AleCarminati/ml-faultdetection-epuck.git
```

Then, by using `cd ml-faultdetection-epuck` you will enter the newly downloaded folder. 

To build the executable, use the following list of commands:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Configuration

For end users: the main settings of the experiment can be found in the file `experiments/fault_detection.argos`. Attention: modifying the parameters of the robots and the environment could invalidate the performances of the fault detection algorithm.

For developers: the parameters of the fault detection algorithm can be found in the file `config.ini`. Modify them only if you know what you are doing!

## Run

To run the experiment, from the terminal in the root folder write:

```bash
argos3 -c experiments/fault_detection.argos
```

When you modify the code, from the root folder run the command:

```bash
./execute.sh
```

which recompiles the code and then executes the simulation.

To run one experiment for each type of fault, from the root folder run the command:

```bash
./execute_all_faults.sh
```

 