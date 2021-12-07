%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                 %
% Version Name: QuadcopterTubeMPC_v2     %
% Date: 12/6/2021                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Setup
mpcverbosity off;
warning('off','all');

%% Setup Simulation Variables
% Here we setup the simulation parameters for at Linear Tube MPC. The Plant
% model is based on a Parrot Mambo Drone Quadcopter for a set point that
% allows the drone to hover at 2m. 

[TubePlant,TubeMPCobj,TubeXmpc,TubeMPCopt,Tubemv] = LinerMpcSetup(SimParams);
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
SimResult.Tube.StateTraj = [EstimTrajPrev];
SimResult.Tube.ControlInput = [ThrustRef];

%TubeMPCInfo
TubeMPC = struct();
TubeMPC.EstimTraj = EstimTrajPrev;
TubeMPC.EstimTrajPrev = EstimTrajPrev;
TubeMPC.ThrustRef = Tubemv;
TubeMPC.RefTraj = EstimTrajPrev;

Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0;0;0];
Noise.ControlNoise = [0;0;0;0];
%% Run Simulation
% At each step of the simulation we get the trajectory parameters from the
% Nominal MPC, Collect the Noise data at the current position, run the MPC,
% propogate the system and collect the data

hbar = waitbar(0, 'Tube MPC Simulation Progress');
for SimIteration = 1:(SimParams.Duration/SimParams.dt) 
    RefTrajNominal = SimResult.Nominal.StateTraj(:,SimIteration:SimIteration+SimParams.PredictionHorizon);
    Noise = DisturbanceMain(SimIteration, SimParams, TubeMPC, plantmodels, Uncertainty_data);

    TubePlant = Noise.PlantModel;
    TubeMPCopt.MVTarget = SimResult.Nominal.ControlInput(:,SimIteration+1);
    TubeMPC = LinearTubeMPCFunction(TubeMPC,RefTrajNominal,TubePlant,TubeMPCobj,TubeXmpc,TubeMPCopt, Noise, SimParams);
    
    % Store Data from Simulation
    SimResult.Tube.StateTraj(:,SimIteration+1) = TubeMPC.EstimTraj;
    SimResult.Tube.ControlInput(:,SimIteration+1) = TubeMPC.ThrustRef;
    
    waitbar(SimIteration*SimParams.dt/SimParams.Duration,hbar)
end
close(hbar);
disp("Tube MPC Simulation Finnished")

%% Save Data
save('SimResult.mat','SimResult');
disp("Tube MPC Simulation Data Saved");