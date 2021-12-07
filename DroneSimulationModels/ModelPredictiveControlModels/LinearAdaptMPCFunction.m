function AdaptMPC = LinearAdaptMPCFunction(AdaptMPC,RefTraj,plant,MPCobj,xmpc,MPCopt, SimParams, Iteration, Noise)
% Universal Variables
UMin = SimParams.UMin;
UMax = SimParams.UMax;
gamma = SimParams.gamma;
Kmax = SimParams.Kmax;
%% Adaptive Variables
% In this section we use a single layer network and use that information to
% determine the adaptive control input based on the current position of the
% system

EstimTrajPrev = AdaptMPC.EstimTraj;
LastMV = AdaptMPC.ThrustRef;
K_a = AdaptMPC.Ka;

if Iteration == 1
    sigma = zeros(SimParams.n2+1,1);
else
    sigma = sdash_rbf(EstimTrajPrev,SimParams.rbf_c,SimParams.rbf_mu,SimParams.bw,SimParams.n2);
end

% Calculate the adaptive control input and adjust the tubes of the MPC to
% determine the next control input.
u_a = -K_a'*sigma;
uub = UMax + abs(u_a');
ulb = UMin - abs(u_a');

for i = 1:length(uub)
    MPCobj.MV(i).Min = ulb(i);
    MPCobj.MV(i).Max = uub(i);
end

%% MPC Control and Propogate Movement
y = plant.C*EstimTrajPrev; % calculate plant output
ThrustRef = mpcmove(MPCobj,xmpc,y,RefTraj',[],MPCopt); % calculate optimal mpc move
switch SimParams.DisturbanceCase
    case "Damaged Blade - No Wind Bias"
        ThrustRef = ThrustRef*0.5;
end
EstimTraj = plant.A*(EstimTrajPrev+Noise.StateNoise)+plant.B*(ThrustRef+u_a+Noise.ControlNoise); % update plant state

%% Update the Adaptive Gains 
% We update the adaptive gaines for the system based on the error of the reference trajectory and the estimated trajectory calculated
% we alter the first iteration because there is no difference in position
% and the norm of the value sigma will be 0

error = EstimTraj-RefTraj(:,1);
if Iteration == 1
   delta1 = gamma * sigma * (pinv(plant.B)*(error))';
   delta2 = -(gamma * sigma * (u_a+Noise.ControlNoise)');
else
    delta1 = (gamma * sigma * (pinv(plant.B)*(error))')/norm(sigma)^2;
    delta2 = -(gamma * sigma * (u_a+Noise.ControlNoise)')/norm(sigma)^2;
end
%delta1 = 0;
delta2 = 0;

K_a_bar = K_a + (delta1 + delta2);
if norm(K_a_bar,1) >= Kmax %Kmax
    K_a = (Kmax/norm(K_a_bar,1))*K_a_bar;
else
    K_a = K_a_bar;
end  

%% Store Results
AdaptMPC = struct();
AdaptMPC.EstimTraj = EstimTraj;
AdaptMPC.ThrustRef = ThrustRef+u_a;
AdaptMPC.Ka = K_a;
AdaptMPC.ua = u_a;
end