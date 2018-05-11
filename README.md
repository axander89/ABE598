# ABE598

This is the work done for the final project for Autonomous Decision Making (ABE598). The scope of the project is to explore path planning algorithms for autonomous systems, in particular RRT and RRT*, and to implement it on a real robot in a simulation engine. One of the contributions of this project is the fully functional open source code for research purposes and for enthusiastic roboticists who do not have access to real physical hardware.

## Introduction

In the ideal world of autonomy complex systems must carry out complex decision-making tasks to accomplish desired goals. One of these essential tasks is path planning. If we know a robot as well as its initial position and orientation, and if we can sense the environment, how can we drive the robot to another goal position and orientation while obeying the rules of the robot and environment, e.g. not colliding with obstacles or with itself? This is the problem that this project solves and implements with the aid of rapidly exploring random trees, a sampling-based algorithm

## Preliminary Material

### Forward Kinematics

To solve for the forward kinematics of the Jaco the product of exponentials is used. 

![alt text](https://github.com/axander89/ABE598/blob/master/imgs/fk.png " remove child scripts")

where[S<sub>i</sub>] is the skew symmetric matrix of the spatial screw vector of joint i, and θ<sub>i</sub> is the angle of rotation for joint i. More precisely,

![alt text](https://github.com/axander89/ABE598/blob/master/imgs/fk2.png " remove child scripts")
