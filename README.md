# DeepMPC
 Tube based Model Predictive Control that utilize Neural Networks to improve performance with adaptive control inputs.

## Table of contents
* [General Info](#General-Info)
* [Setup](#Setup)
* [DroneSimulationMain](#DroneSimulationMain)
* [QuadDataProcessing](#QuadDataProcessing)
* [Current Results](#Current-Results)
* [Acknolwedgments](#Ackowledgments)

## General Info
The ReadMe will be updated with more information with further development of the DeepMPC software, and when my Thesis has been fully reviewed and submitted.
## Setup
In order to run the Deep MPC program as well as the other aspects of the Drone Simulation code you will need the following MathWorks MATLAB toolboxes:
* [Model Predictive Control Toolbox](https://www.mathworks.com/products/model-predictive-control.html)
* [Symbolic Math Toolbox](https://www.mathworks.com/products/symbolic.html)
* [Optimization Toolbox](https://www.mathworks.com/products/optimization.html)
* [Deep Learning Toolbox](https://www.mathworks.com/products/deep-learning.html)

In addition to these packages you will need the [tbxmanager](https://www.tbxmanager.com/package/index)
## DroneSimulationMain
The DroneSimulationMain.m script runs the MATLAB Scripts for the Tube-Based MPC, Adaptive MPC, and Deep MPC control systems for a model of a Parrot Mambo Drone. To try out different Disturbance or Trajectory Cases modify the SimulationParameters.m file. The disturbance cases created are variations of Wind Biases, Damaged Blades, Center of Mass Shifts, and Mass changes. The trajectory cases are for Hover, Circle, and Figure 8 Manuevers. 
## QuadDataProcessing
The QuadDataProcessing.m script runs the MATLAB Scripts to generate the plots and processed data for the data collected in the DroneSimulationMain.m script. This includes plots of the algorithms following the manuver, position and orientation trajectory, and the values for the deviation from the trajectory.
## Current Results
![Fig8-Nominal](/ProcessedResults/PNG%20Files/Figure8/Figure8_Nominal_Full_Trajectory.png)
### Fig-8 Nominal

## Acknowledgments
I would like to thank the people at Mathworks for their various toolboxes, the developers of the tbxmanager software, and my colleague Mateus Valverde for all of their efforts in their software development that have enabled the development of the DeepMPC Scripts.
