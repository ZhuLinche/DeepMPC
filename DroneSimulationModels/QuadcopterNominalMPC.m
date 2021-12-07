%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                    %
% Version Name: QuadcopterNomianlMPC_v2     %
% Date: 12/06/2021                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Setup
%Setups Matlab to reset variables and workspace
% clear all;
% close all;
mpcverbosity off;
warning('off','all');

%% Setup the System Modek
%These lines setup the system model for the quadcopter using the function
%get QuadrotorDynamicsandJacobian. The outputs of these files generate the
%files QuadrotorStateFcn.m and QuadrotorStateHacobianFcn.m for different
%scenarios. If you make changes make sure to regenerate these files by
%either uncommenting the appropriate line or running your code to generate
%these files.

% getQuadrotorDynamicsAndJacobian;
% getQuadrotorDynamicsAndJacobianBladeDamage;
%getQuadrotorDynamicsAndJacobianCOMShift;
%getQuadrotorDynamicsAndJacobianMassCapacity;
%getQuadrotorDynamicsAndJacobianHarshCOMShift;

%% Setup Simulation Variables
SimParams = SimulationParameters();
[NominalPlant,NominalMPCobj,NominalXmpc] = LinerNominalMpcSetup(SimParams);

% Setup Initial Values for Simulation
EstimTrajPrev = [0;0;0;0;0;0;0;0;0;0;0;0];
RefTraj = [0;0;0;0;0;0;0;0;0;0;0;0];

%Setup Data Structure for Plotting and Analysis
SimResult = struct();
SimResult.Nominal.StateTraj = [EstimTrajPrev];
SimResult.Nominal.ControlInput = [zeros(4,1)];
SimResult.Nominal.RefTraj = [EstimTrajPrev];

%NominalMPCInfo
NominalMPC = struct();
NominalMPC.EstimTraj = EstimTrajPrev;
NominalMPC.ThrustRef = zeros(1,4);
NominalMPC.CpuTime = [ ];
NominalMPC.CpuTotalTime = [ ];

%% Nominal MPC
hbar = waitbar(0,'Nominal MPC Simulation Progress');
terminalStart = cputime;
for NominalIteration = 1:((SimParams.Duration/SimParams.dt)+SimParams.PredictionHorizon)
    NominalTime = linspace(SimParams.dt*(NominalIteration-1), (NominalIteration+SimParams.PredictionHorizon-1)*SimParams.dt,SimParams.PredictionHorizon);
    RefTraj = QuadrotorReferenceTrajectory(NominalTime, SimParams); 
    
    NominalTStart = cputime;
    NominalMPC = LinearNominalMPCFunction(NominalMPC,RefTraj,NominalPlant,NominalMPCobj,NominalXmpc);
    NominalTEnd = cputime-NominalTStart;
    
    SimResult.Nominal.StateTraj(:, NominalIteration+1) = NominalMPC.EstimTraj;
    SimResult.Nominal.ControlInput(:, NominalIteration+1) = NominalMPC.ThrustRef;
    SimResult.Nominal.RefTraj(:, NominalIteration+1) = RefTraj(:, 1);
    NominalMPC.CpuTime(:,NominalIteration) = NominalTEnd;
    
    waitbar(NominalIteration*SimParams.dt/SimParams.Duration,hbar);
end
terminalEnd = cputime-terminalStart;
NominalMPC.CpuTotalTime(:,1) = terminalEnd;

close(hbar);
disp("Nominal Trajectory Planned")

%% Save Data
save('SimResult.mat','SimResult');
disp("Nominal MPC Data Saved");