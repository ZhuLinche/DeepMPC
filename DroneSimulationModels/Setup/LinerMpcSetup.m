%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: LinearMpcSetup                    %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [plant,MPCobj,xmpc,MPCopt,mv] = LinerMpcSetup(SimParams)
% Depending on the disturbance case the setpoint at which the plant model
% is linearized needs to change. 
switch SimParams.DisturbanceCase
    case "Mass Change"
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [15.3568 15.3568 15.3568 15.3568];
        [A,B] = QuadrotorStateJacobianFcnMassCapacity(xk',uk');
    case "High Mass COM Shift"   
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [15.3568 15.3568 15.3568 15.3568];
        [A,B] = QuadrotorStateJacobianFcnMassCapacity(xk',uk');
    otherwise    
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [14.4 14.4 14.4 14.4];
        [A,B] = QuadrotorStateJacobianFcn(xk',uk');
end

%Parameters for the MPC model
Ts = SimParams.dt; %sample time
Nhori = SimParams.ControlHorizon; %control horizon
Nsim = SimParams.PredictionHorizon; %prediction horizon

%CSTR = ss(A,B,C,D);
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;

MPCobj = mpc(plant,Ts, Nsim, Nhori);
MPCobj.PredictionHorizon = Nsim;
MPCobj.ControlHorizon = Nhori;
% Set the boundaries of the MPc model for output and input variables
MPCobj.MV = struct('Max',{SimParams.UMax(1),SimParams.UMax(2),SimParams.UMax(3),SimParams.UMax(4)}...
    ,'Min',{SimParams.UMin(1),SimParams.UMin(2),SimParams.UMin(3),SimParams.UMin(4)});
MPCobj.OV = struct('Min',{-SimParams.XLim(1),-SimParams.XLim(2),-SimParams.XLim(3),-SimParams.XLim(4),-SimParams.XLim(5),-SimParams.XLim(6) ...
    ,-SimParams.XLim(7),-SimParams.XLim(8),-SimParams.XLim(9),-SimParams.XLim(10),-SimParams.XLim(11),-SimParams.XLim(12)} ...
    ,'Max',{SimParams.XLim(1),SimParams.XLim(2),SimParams.XLim(3),SimParams.XLim(4),SimParams.XLim(5),SimParams.XLim(6) ...
    ,SimParams.XLim(7),SimParams.XLim(8),SimParams.XLim(9),SimParams.XLim(10),SimParams.XLim(11),SimParams.XLim(12)});
%Set the weights (Q, R) for the Cost function of the MPC
MPCobj.Weights.OutputVariables = SimParams.OVWeights;
MPCobj.Weights.ManipulatedVariables = SimParams.MVWeights;

xmpc = mpcstate(MPCobj);
MPCopt = mpcmoveopt;
mv = [0 0 0 0];
end