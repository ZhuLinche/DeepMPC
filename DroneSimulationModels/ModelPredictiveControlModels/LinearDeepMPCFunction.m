function [DeepMPC, inner_net]  = LinearDeepMPCFunction(DeepMPC,RefTraj,plant,MPCobj,xmpc,MPCopt, SimParams, inner_net, Iteration, Noise)
 % Universal Variables
UMin = SimParams.UMin;
UMax = SimParams.UMax;
gamma = SimParams.gamma;
Kmax = SimParams.Kmax;

%% Adaptive Variables
% In this section we use a multi layer neural network and use that information to
% determine the adaptive control input based on the current position of the
% system

EstimTrajPrev = DeepMPC.EstimTraj;
LastMV = DeepMPC.ThrustRef;
K_a = DeepMPC.Ka;
freq_ratio = DeepMPC.freq_ratio;

sigma = inner_net.forward(EstimTrajPrev);

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

%% Update the Adaptive Gains and NN
% We update the adaptive gaines for the system based on the error of the reference trajectory and the estimated trajectory calculated

error = EstimTraj-RefTraj(:,1);
delta1 = (gamma * sigma * (pinv(plant.B)*(error))')/norm(sigma)^2;
delta2 = 0;

K_a_bar = K_a + (delta1 + delta2);
if norm(K_a_bar,1) >= Kmax %Kmax
    K_a = (Kmax/norm(K_a_bar,1))*K_a_bar;
else
    K_a = K_a_bar;
end  

% If we don't tell the NN to stop training update the NN inner layers at the frequency set
% in the parameters
if (DeepMPC.freeze_after==-1) || (Iteration<DeepMPC.freeze_after)
    inner_net = inner_net.add_label(EstimTrajPrev, u_a);
    if mod(Iteration,freq_ratio) == 0
        inner_net = inner_net.train_layers(K_a);
    end
end
    
%Store Results
DeepMPC.EstimTraj = EstimTraj;
DeepMPC.ThrustRef = ThrustRef+u_a;
DeepMPC.Ka = K_a;
DeepMPC.ua = u_a;
end