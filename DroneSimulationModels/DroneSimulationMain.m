%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: DroneSimulationMain_v1_2 - Linear %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Instructions for Simulation
% Edit Line 31 for the simulation you are trying to run. This will be
% partly based on the Setup you create
clear all;
close all;
clearvars -global;
% Add Folders to path incase MatLab is just loaded
currentFolder = pwd;
addpath([currentFolder,'\Disturbrance Cases']);
addpath([currentFolder,'\ModelPredictiveControlModels']);
addpath([currentFolder,'\NeuralNetworkModels']);
addpath([currentFolder,'\Results']);
addpath([currentFolder,'\Setup']);
addpath([currentFolder,'\SimulationModels']);
addpath([currentFolder,'\TrajectoryModels']);
addpath(genpath([currentFolder,'\tbxmanager']));

%% Run Simulations for Nominal, Tube, Adaptive, and Deep MPC
run QuadcopterNominalMPC.m;
run QuadcopterTubeMPC.m
run QuadcopterAdaptMPC.m;
run QuadcopterDeepMPC.m;

%% Generate Plots and Deviation Data and Save Plots and Data in Session Folder
load('SimResult.mat');
FileNameRaw = 'Session1A_Figure8_NominalTrajectory';
FileName = [FileNameRaw,'.mat'];
save(FileName, 'SimResult', 'SimParams');
delete('SimResult.mat');

fclose('all');
movefile(FileName,[currentFolder,'\Results']);

%% Clean up at the end
clear all;