# ABE598

This is the work done for the final project for Autonomous Decision Making (ABE598). The scope of the project is to explore path planning algorithms for autonomous systems, in particular RRT and RRT*, and to implement it on a real robot in a simulation engine. One of the contributions of this project is the fully functional open source code for research purposes and for enthusiastic roboticists who do not have access to real physical hardware.

# Simulations

## Prerequisites

1. MATLAB
The code has been run in [MATLAB_R2017a](https://www.mathworks.com/downloads/), but it should be compatible with later versions. 

2. V-REP
Download V-REP PRO EDU from the Coppelia Robotics website: [here](http://www.coppeliarobotics.com)


## Running Simulations

1. `$ git clone https://github.com/axander89/ABE598.git`
2. `$ cp <vrep_directory>/vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib ./ABE598/src/code/`
3. `$ cd /ABE598/src/code/`
3. open another terminal and run
4. `$ cd <vrep_directory>/vrep/`
5. `$ ./vrep.app/Contents/MacOS/vrep`
6. from original terminal, i.e. <ABE598_directory>/ABE598/src/code/, open and run 'ABE598_Planning_Simulation.m'
7. watch demo 

