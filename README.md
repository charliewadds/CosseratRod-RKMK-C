# SuRFR: A fast recursive simulator for soft manipulators with discrete joints on SE(3)

This project is a C implementation of a MATLAB-based research paper. The goal of this project is to improve runtime performance by converting the original MATLAB code to C. 


## Introduction

This repository contains the C implementation of the SuRFR: A fast recursive simulator for soft manipulators with discrete joints on SE(3) paper. The original project was developed in MATLAB as part of a paper by Hossain Samei and Robin Chhabra. This implementation aims to enhance the runtime efficiency and performance an allow for faster than real time execution. This is a first proof of concept version, as such there are likely bugs, there are memory leaks caused by the Robot Strucs because it will be replaced in the next version. Also there is very little documentation and Comments. The final version will be on my github (http://github.com/charliewadds) page and will be called SuRFR.

## Usage

To run the project, you will need to make sure the following dependencies are installed:
- CMake
- C Compiler
- LAPACK
- BLAS
- GSL (Linker in the Cmake will need to be updated for your installation directory)

To build the LevMar library, run the following commands from the project root directory:

``` cd levmar-2.6```

``` mkdir build && cd build```

``` cmake ..```

``` make```

To build the project, run the following commands from the project root directory:

``` mkdir build && cd build``` to create a build directory

``` cmake ..``` to generate the makefile

``` make``` to build the project.

To run the project, run the following command from the build directory:

``` ./FwdDyn``` for forward dynamics.
    
``` ./InvDyn``` for inverse dynamics.
Inverse dynamics will need to be run first to generate the data for the forward dynamics. If you wish to re-run the Inverse dynamics with new data, the ControlSim.csv file will need to be deleted.

    


## Links
- The original matlab code can be found [here](https://github.com/HSamei/CosseratRod-RKMK.git).
- The paper can be found [here](https://www.researchgate.net/publication/371658455_SuRFR_A_Fast_Recursive_Simulator_for_Soft_Manipulators_with_Discrete_Joints_on_SE3)
