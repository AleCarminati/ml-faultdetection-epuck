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

### Single fault

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

### Multi fault

To run the experiment with more than one faulty robot, from the terminal in the root folder write:

```bash
argos3 -c experiments/fault_detection_multifault.argos
```

When you modify the code, from the root folder run the command:

```bash
./execute.sh m
```

which recompiles the code and then executes the simulation.

 ## Contributing

Before committing any changes, be sure to have installed the pre-commit hooks by running the following command from the root folder:

```bash
./bash/setup_pre_commit.sh
```

The pre-commit hooks contain some automatic checks to improve readability of the code. In particular, it uses the `clang` package.

## Milestones

Commit [19f511f](https://github.com/AleCarminati/ml-faultdetection-epuck/commit/19f511f52f5c2740eff590ec553d2d9f3853ff14): this version of the software has been used to generate training data and to test the fault detection system when a single robot is fault, obtaining the results that are displayed in the master's thesis.

Commit [c503768](https://github.com/AleCarminati/ml-faultdetection-epuck/commit/c503768388831d439c259403915dbe59656d0d71): this version of the software has been used to test the fault detection system when multiple robots are fault, obtaining the results that are displayed in the master's thesis.
