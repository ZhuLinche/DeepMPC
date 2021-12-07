%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                 %
% Version Name: QuadcopterDeepMPC_v2     %
% Date: 12/6/2021                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Setup
mpcverbosity off;
warning('off','all');

%% Setup Simulation Variables
% Here we setup the simulation parameters for at Linear Tube MPC. The Plant
% model is based on a Parrot Mambo Drone Quadcopter for a set point that
% allows the drone to hover at 2m. 

[DeepPlant,DeepMPCobj,DeepXmpc,DeepMPCopt,Deepmv] = LinerMpcSetup(SimParams);
inner_net = inner_layers(SimParams);
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
SimResult.Deep.StateTraj = [EstimTrajPrev];
SimResult.Deep.ControlInput = [ThrustRef];
SimResult.Deep.AdaptiveControlInput = [ThrustRef];

%TubeMPCInfo
DeepMPC = struct();
DeepMPC.EstimTraj = EstimTrajPrev;
DeepMPC.EstimTrajPrev = EstimTrajPrev;
DeepMPC.EstimTrajExpected = EstimTrajPrev;
DeepMPC.ThrustRef = Deepmv;
DeepMPC.RefTraj = EstimTrajPrev;
DeepMPC.Ka = zeros(SimParams.n2+1,SimParams.nu);
DeepMPC.freq_ratio = SimParams.freq_ratio;
DeepMPC.freeze_after = SimParams.freeze_after;

Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0.1;0;0];
Noise.ControlNoise = [0;0;0;0];

%% Run Simulation
% At each step of the simulation we get the trajectory parameters from the
% Nominal MPC, Collect the Noise data at the current position, run the MPC,
% propogate the system and collect the data

hbar = waitbar(0, 'Deep MPC Simulation Progress');
for SimIteration = 1:(SimParams.Duration/SimParams.dt) 
    RefTrajNominal = SimResult.Nominal.StateTraj(:,SimIteration:SimIteration+SimParams.PredictionHorizon);
    Noise = DisturbanceMain(SimIteration, SimParams, DeepMPC, plantmodels, Uncertainty_data);

    DeepPlant = Noise.PlantModel;
    DeepMPCopt.MVTarget = SimResult.Nominal.ControlInput(:,SimIteration+1);
    [DeepMPC, inner_net]  = LinearDeepMPCFunction(DeepMPC,RefTrajNominal,DeepPlant,DeepMPCobj,DeepXmpc,DeepMPCopt, SimParams, inner_net, SimIteration, Noise);
   
    % Store Data from Simulation
    SimResult.Deep.StateTraj(:,SimIteration+1) = DeepMPC.EstimTraj;
    SimResult.Deep.ControlInput(:,SimIteration+1) = DeepMPC.ThrustRef;
    SimResult.Deep.AdaptiveControlInput(:,SimIteration+1) = DeepMPC.ThrustRef;
    
    waitbar(SimIteration*SimParams.dt/SimParams.Duration,hbar)
end
close(hbar);
disp("Deep MPC Simulation Finnished")

%% Save Data
save('SimResult.mat','SimResult');
disp("Deep MPC Simulation Data Saved");