%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                  %
% Version Name: QuadcopterAdaptMPC_v2     %
% Date: 12/6/2021                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Setup
mpcverbosity off;
warning('off','all');

%% Setup Simulation Variables
% Here we setup the simulation parameters for at Linear Tube MPC. The Plant
% model is based on a Parrot Mambo Drone Quadcopter for a set point that
% allows the drone to hover at 2m. 

[AdaptPlant,AdaptMPCobj,AdaptXmpc,AdaptMPCopt,Adaptmv] = LinerMpcSetup(SimParams);
plantmodels = PlantModelSetup(SimParams);

% Setup Initial Values for Simulation
EstimTrajPrev = [0;0;0;0;0;0;0;0;0;0;0;0];
RefTraj = [0;0;0;0;0;0;0;0;0;0;0;0];
ThrustRef = zeros(4,1);
EstimTrajTubejPrev = EstimTrajPrev;
ThrustRefTube = ThrustRef;
VelocityBodyTube = zeros(3,1);

% Setup Data Structure for Plotting and Analysis
load('SimResult.mat','SimResult');
load('Uncertainty_data.mat');
SimResult.Adapt.StateTraj = [EstimTrajPrev];
SimResult.Adapt.ControlInput = [ThrustRef];
SimResult.Adapt.AdaptiveControlInput = [ThrustRef];

%TubeMPCInfo
AdaptMPC = struct();
AdaptMPC.EstimTraj = EstimTrajPrev;
AdaptMPC.EstimTrajPrev = EstimTrajPrev;
AdaptMPC.EstimTrajExpected = EstimTrajPrev;
AdaptMPC.ThrustRef = Adaptmv;
AdaptMPC.RefTraj = EstimTrajPrev;
AdaptMPC.Ka = zeros(SimParams.n2+1,1);
AdaptMPC.freq_ratio = 10;
AdaptMPC.freeze_after = 0;

Noise = struct();
Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0.1;0;0];
Noise.ControlNoise = [0;0;0;0];

%% Run Simulation
% At each step of the simulation we get the trajectory parameters from the
% Nominal MPC, Collect the Noise data at the current position, run the MPC,
% propogate the system and collect the data

hbar = waitbar(0, 'Adaptive MPC Simulation Progress');
for SimIteration = 1:(SimParams.Duration/SimParams.dt) 
    RefTrajNominal = SimResult.Nominal.StateTraj(:,SimIteration:SimIteration+SimParams.PredictionHorizon);
    Noise = DisturbanceMain(SimIteration, SimParams, AdaptMPC, plantmodels, Uncertainty_data);
    
    AdaptPlant = Noise.PlantModel;
    SimResult.Nominal.ControlInput(:,SimIteration+1);
    AdaptMPC = LinearAdaptMPCFunction(AdaptMPC,RefTrajNominal,AdaptPlant,AdaptMPCobj,AdaptXmpc,AdaptMPCopt, SimParams, SimIteration, Noise);
    
    % Store Data from Simulation
    SimResult.Adapt.StateTraj(:,SimIteration+1) = AdaptMPC.EstimTraj;
    SimResult.Adapt.ControlInput(:,SimIteration+1) = AdaptMPC.ThrustRef;
    SimResult.Adapt.AdaptiveControlInput(:,SimIteration+1) = AdaptMPC.ThrustRef;
    
    waitbar(SimIteration*SimParams.dt/SimParams.Duration,hbar)
end
close(hbar);
disp("Adapt MPC Simulation Finnished")

%% Save Data
save('SimResult.mat','SimResult');
disp("Adapt MPC Simulation Data Saved");