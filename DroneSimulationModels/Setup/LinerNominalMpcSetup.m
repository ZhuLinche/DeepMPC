%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: LinearNominalMpcSetup             %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [plant,MPCobj,xmpc] = LinerMpcSetup(SimParams)
% The parameters for MPC setup are for the nominal case without disturbance

xk = [0 0 1 0 0 0 0 0 0 0 0 0];
uk = [14.4 14.4 14.4 14.4];

[A,B] = QuadrotorStateJacobianFcn(xk',uk');
Ts = SimParams.dt;
Nhori = SimParams.ControlHorizon;
Nsim = SimParams.PredictionHorizon;

%CSTR = ss(A,B,C,D);
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;

Q = diag(SimParams.OVWeights);
R = diag(SimParams.MVWeights);

[K Qf] = dlqr(A, B, Q, R);

Xc = SimParams.Xc;
Uc_prime = SimParams.Uc_prime;
W = SimParams.W;

% Use the maximum and minimum values of the constructed polyhedron of
% disturbance to find the upper and lower bounds of our tube for the
% Nominal MPC to follow.

Z_approx_max = max(W.V)';
Z_approx_min = min(W.V)';
for n = 1:4
   Z_approx_max = Z_approx_max + (plant.A-plant.B*K)^n*max(W.V)';
end
xlb_nominal = min(Xc.V)'-abs(Z_approx_max);
xub_nominal = abs(max(Xc.V)'+abs(Z_approx_max));
ulb_nominal = min(Uc_prime.V)'-K*abs(Z_approx_max);
uub_nominal = max(Uc_prime.V)'+K*abs(Z_approx_max);

MPCobj = mpc(plant,Ts, Nsim, Nhori);
MPCobj.PredictionHorizon = Nsim;
MPCobj.ControlHorizon = Nhori;
% MPCobj.Model.Plant.OutputUnit = {'Deg','Deg/s'};
MPCobj.MV = struct('Min',{ulb_nominal(1),ulb_nominal(2),ulb_nominal(3),ulb_nominal(4)}...
    ,'Max',{uub_nominal(1),uub_nominal(2),uub_nominal(3),uub_nominal(4)});
MPCobj.OV = struct('Min',{xlb_nominal(1),xlb_nominal(2),xlb_nominal(3),xlb_nominal(4),xlb_nominal(5),xlb_nominal(6)...
	,xlb_nominal(7),xlb_nominal(8),xlb_nominal(9),xlb_nominal(10),xlb_nominal(11),xlb_nominal(12)} ...
	,'Max',{xub_nominal(1),xub_nominal(2),xub_nominal(3),xub_nominal(4),xub_nominal(5),xub_nominal(6)...
	,xub_nominal(7),xub_nominal(8),xub_nominal(9),xub_nominal(10),xub_nominal(11),xub_nominal(12)});
%MPCobj.Model.Nominal.U = pars.MVNominal;
MPCobj.Weights.OutputVariables = SimParams.OVWeights;
MPCobj.Weights.ManipulatedVariables = SimParams.MVWeights;

xmpc = mpcstate(MPCobj);
end