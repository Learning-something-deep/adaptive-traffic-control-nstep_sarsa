# Adaptive traffic control system using Reinforcement Learning

Copyright 2020, Akshay Kekuda, R. Anirudh, Mithun Krishnan. All rights reserved.

This project implements n-step SARSA algorithm to tackle the traffic congestion problem. The dependencies for this project are listed under the requirements.txt file. Make sure that the required dependencies are installed.

We use SUMO[Simulation of Urban MObility] V1.6 package to simulate traffic patterns on which the algorithm has been tested. Details of SUMO installation can be found at https://sumo.dlr.de/docs/Installing.html

We have used PyCharm and Anaconda for developement of this project. Any IDEs can be used, provided that the dependencies are properly installed in your respective environments. The OS can be Windows/Linux

The project folder contains two directories: src and scripts
* The src directory contains all the source codes needed to run the project
* The scripts directory contains SUMO files describing the road network and routes. Also output files from SUMO will be generated in this directory.


## Running n-step SARSA
The src folder contains the following files and sub-directories:
1. main.py: This script contains the main function of the project. It includes calls to static_signalling(), lqf(), sarsa_nstep_diff_train(), sarsa_nstep_diff_live() and plot_all_metrics() functions.
2. lqf.py: This script contains the module that implements the Longest Queue First algorithm.
3. static_signalling.py: This script contains the module that implements the Static Signalling [SS] algorithm
4. sarsa_nstep.py: This script constains the following modules:
	1. sarsa_nstep_diff_train(): This module implements n-step SARSA algorithm as described in Richard S. Sutton's Reinforcement Learning, 2nd Ed book. 
	2. sarsa_nstep_diff_live(): This module makes use of the trained weights from sarsa_nstep_diff_train() module to test the performance of the n-step SARSA algorithm
5. sim_environment.py: This script contains modules that model the traffic network. It outputs states, reward for an action and network related files essential for simulating the traffic.
6. plot_metrics.py: This file contains modules that plot the following metrics: Queue Length, Waiting Time, Time Loss and Dispersion Time. It also saves the metrics into the "datapoints" directory present in the src folder
7. test.py: This file plots the primary performance graphs of the n-step SARSA algorithm. It compares LQF, SS and n-step SARSA algorithm using the Queue Length, Waiting Time, Time Loss and Dispersion Time metrics.
8. datapoints: This directory saves the Queue Length, Waiting Time, Time Loss and Dispersion Time metrics. Ensure that the directory is present inside the src folder

### Procedure to run the project
1. Set the n,c and Nruns parameters in the main.py file. These values can be changed to test the n-step SARSA algorithm. We found optimal performance with n=3 and c=2. The Nruns parameter runs training/live for "Nruns" trials. Recommended minimum Nruns is 20
2. Run main.py. This will implement SS Algo for "Nruns" trials and LQF Algo for "Nruns" trials. Then training for n-step SARSA will start. Based on the trained weights, SARSA Live will run for "Nruns" trials. All the performance metrics will be stored into the "/datapoints" sub-directory at the end of "Nruns" trial.
3. Set the n and c values that were used in the previous step in the test.py file and run test.py to get the performance graphs.


