%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: SimulationParameters              %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function SimParams = SimulationParameters()
    SimParams = struct();
    
    %% Simulation and Vehicle Paramters
    SimParams.dt = 0.05;
    SimParams.Mass = 0.063;
    SimParams.g = 9.81;
    SimParams.Jx = 0.0000582857;
    SimParams.Jy = 0.0000716914;
    SimParams.Jz = 0.0001;
    
    %% PID Control Parameters
    %Yaw Parameters
    SimParams.PotentialYaw = 0.0004;
    SimParams.DerivativeYaw = 0.03*0.0004;
    %Pitch Parameters
    SimParams.PotentialPitch = 0.0013;
    SimParams.IntegralPitch = 0.001;
    SimParams.DerivativePitch = 0.0002;
    SimParams.IntegrationValPitch = 0;
    %Roll Parameters
    SimParams.PotentialRoll = 0.001;
    SimParams.IntegralRoll = 0.001;
    SimParams.DerivativeRoll = 0.00028;
    SimParams.IntegrationValRoll = 0;
    %X Parameters
    SimParams.PotentialX = -0.044;
    SimParams.IntegralX = 0;
    SimParams.DerivativeX = -0.035;
    SimParams.IntegrationValX = 0;
    %Y Parameters
    SimParams.PotentialY = -0.044;
    SimParams.IntegralY = 0;
    SimParams.DerivativeY = -0.035;
    SimParams.IntegrationValY = 0;
    %Z Parameters
    SimParams.PotentialZ = 0.08;
    SimParams.DerivativeZ = 0.05;
    
    SimParams.ThrustGain = 1;
    
    %% MPC Setup
    SimParams.PredictionHorizon = 10;
    SimParams.ControlHorizon = 2;
    SimParams.nx = 12;
    SimParams.nu = 4; 
    SimParams.OVWeights = [1 1 1 0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01]; %Output Variable Weights (xd)
    SimParams.MVWeights = [0.1 0.1 0.1 0.1]; %Manipulated Variable Weights (u)
    
    % Max Control Input variables
    SimParams.PitchCmdMax = 30;
    SimParams.RollCmdMax = 30;
    SimParams.YawCmdMax = 30;
    SimParams.ThrustCmdMax = 30;
    
    % Max State variables
    SimParams.XMax = 4;
    SimParams.YMax = 4;
    SimParams.ZMax = 4;
    SimParams.PhiMax = pi/4;
    SimParams.ThetaMax = pi/4;
    SimParams.PsiMax = pi/4;
    SimParams.UMax = 5;
    SimParams.VMax = 5;
    SimParams.WMax = 5;
    SimParams.PMax = pi/2;
    SimParams.QMax = pi/2;
    SimParams.RMax = pi/2;
    
    % Vectors for the limits of the state and control variables
    SimParams.XLim = [SimParams.XMax,SimParams.YMax,SimParams.ZMax,SimParams.PhiMax,SimParams.ThetaMax,SimParams.PsiMax...
        ,SimParams.UMax,SimParams.VMax,SimParams.WMax,SimParams.PMax,SimParams.QMax,SimParams.RMax];
    SimParams.ULim = [SimParams.PitchCmdMax,SimParams.RollCmdMax,SimParams.YawCmdMax,SimParams.ThrustCmdMax];
    
    % Construct a polyhedron of all possible state combinations
    SimParams.Xc_vertex = combinations(SimParams.XLim);
    SimParams.Xc = Polyhedron(SimParams.Xc_vertex);
    
    % Parameters for the quadcopter constants
    SimParams.kVal = 0.0107;
    SimParams.lVal = 0.0624;
    SimParams.bVal = 0.78264*10^-3;
    
    %% Adaptive MPC Setup    
    % parameters for adaptive control
    SimParams.gamma = 0.01;
    SimParams.UMax = [SimParams.PitchCmdMax,SimParams.RollCmdMax,SimParams.YawCmdMax,SimParams.ThrustCmdMax];
    SimParams.UMin = -SimParams.UMax;
    %SimParams.Kmax = 0.1;
    % uncertainties are approximated by a NN of RBF's
    % parameters for RBF
    SimParams.n2=15; % number of neurons
    x_space = [-10 10];
    y_space = [-10 10];
    z_space = [-10 10];
    phi_space = [-pi pi];
    theta_space = [-pi pi];
    psi_space = [-pi pi];
    xd_space = [-10 10];
    yd_space = [-10 10];
    zd_space = [-10 10];
    phid_space = [-pi pi];
    thetad_space = [-pi pi];
    psid_space = [-pi pi];
    SimParams.bw=1; % first element of NNs
    %Distribute centers using a uniform random distribution
    for ii = 2:SimParams.n2+1
        rbf_c(1,ii)=unifrnd(x_space(1),x_space(2));
        rbf_c(2,ii)=unifrnd(y_space(1),y_space(2));
        rbf_c(3,ii)=unifrnd(z_space(1),z_space(2));
        rbf_c(4,ii)=unifrnd(phi_space(1),phi_space(2));
        rbf_c(5,ii)=unifrnd(theta_space(1),theta_space(2));
        rbf_c(6,ii)=unifrnd(psi_space(1),psi_space(2));
        rbf_c(7,ii)=unifrnd(xd_space(1),xd_space(2));
        rbf_c(8,ii)=unifrnd(yd_space(1),yd_space(2));
        rbf_c(9,ii)=unifrnd(zd_space(1),zd_space(2));
        rbf_c(10,ii)=unifrnd(phid_space(1),phid_space(2));
        rbf_c(11,ii)=unifrnd(thetad_space(1),thetad_space(2));
        rbf_c(12,ii)=unifrnd(psid_space(1),psid_space(2));
        rbf_mu(ii)=1;
    end
    SimParams.rbf_c = rbf_c;
    SimParams.rbf_mu = rbf_mu;
    
    %% Deep Params
    %Parameters for Multi-layer network
    SimParams.hidden_size = [15,15];
    SimParams.training_fcn = 'traingdm';
    SimParams.p_max = 70/SimParams.dt;   % Buffer size
    SimParams.samples = 70/SimParams.dt;  % Sample size from buffer
    SimParams.lr = 0.01;     % Learing rate
    SimParams.label_tol = 0.001;
    SimParams.freq_ratio = 10; % How much slower are the inner layers trained in comparison to the outer layers
    SimParams.fix_seed = true;
    
    %% Disturbance Cases
    % This is a list of the disturbance cases for the simulation
     SimParams.DisturbanceCase = "Nominal"; %Nomianl Conditions
    % SimParams.DisturbanceCase = "Low Wind Bias"; %Simulates the quadcopter with a low wind bias
    % SimParams.DisturbanceCase = "Medium Wind Bias"; %Simulates the quadcopter with a medium wind bias
    % SimParams.DisturbanceCase = "High Wind Bias"; %Simulates the quadcopter with a high wind bias
    % SimParams.DisturbanceCase = "Damaged Blade - No Wind Bias"; %Simulates the quadcopter with a damaged blade
    % SimParams.DisturbanceCase = "Damaged Blade - Low Wind Bias"; %Simulates the quadcopter with a damaged bade and low wind bias
    % SimParams.DisturbanceCase = "Damaged Blade - Medium Wind Bias"; %Simulates the quadcopter with a damaged bade and medium wind bias
    % SimParams.DisturbanceCase = "Damaged Blade - High Wind Bias"; %Simulates the quadcopter with a damaged bade and high wind bias
    % SimParams.DisturbanceCase = "COM Shift - No Wind Bias"; %Simulates the quadcopter with a COM shift
    % SimParams.DisturbanceCase = "COM Shift - Low Wind Bias"; %Simulates the quadcopter with a COM shift and a low wind bias
    % SimParams.DisturbanceCase = "COM Shift - Medium Wind Bias"; %Simulates the quadcopter with a COM shift and a medium wind bias
    % SimParams.DisturbanceCase = "COM Shift - High Wind Bias"; %Simulates the quadcopter with a COM shift and a high wind bias
    % SimParams.DisturbanceCase = "Control Disturbance - No Wind Bias"; %Simulates the quadcopter with a Control Disturbance
    % SimParams.DisturbanceCase = "Control Disturbance - Low Wind Bias"; %Simulates the quadcopter with a Control Disturbance and a low wind bias
    % SimParams.DisturbanceCase = "Control Disturbance - Medium Wind Bias"; %Simulates the quadcopter with a Control Disturbance and a medium wind bias
    % SimParams.DisturbanceCase = "Control Disturbance - High Wind Bias"; %Simulates the quadcopter with a Control Disturbance and a high wind bias
    % SimParams.DisturbanceCase = "Low Wind Bias Hover"; %Simulates the quadcopter with a low wind bias
    % SimParams.DisturbanceCase = "Medium Wind Bias Hover"; %Simulates the quadcopter with a medium wind bias
    % SimParams.DisturbanceCase = "High Wind Bias Hover"; %Simulates the quadcopter with a high wind bias   
    % SimParams.DisturbanceCase = "Hover COM Shift High Wind Bias";
    % SimParams.DisturbanceCase = "Hover Blade Damage High Wind Bias"; 
    % SimParams.DisturbanceCase = "Mass Change";
    % SimParams.DisturbanceCase = "High Mass COM Shift";
    %% Trajectory Cases
     SimParams.TrajectoryCase = 'Figure8';
     SimParams.Fig8 = 20;
     SimParams.NumFig8 = 3;
     SimParams.Duration = SimParams.NumFig8*SimParams.Fig8+10;
%      SimParams.TrajectoryCase = 'Circle';
%      SimParams.Circle = 20;
%      SimParams.NumCircles = 3;
%      SimParams.Duration = SimParams.NumCircles*SimParams.Circle+10;
%     SimParams.TrajectoryCase = 'Hover';
%     SimParams.Duration = 30;
       SimParams.freeze_after = SimParams.Duration/SimParams.dt;
    % SimParams.freeze_after = 30;
     
    %% Disturbance Set
    % worst case disturbance and available control authority for constraint
    % tightening
    xk = [0 0 1 0 0 0 0 0 0 0 0 0];
    uk = [14.4 14.4 14.4 14.4];
    [A,B] = QuadrotorStateJacobianFcn(xk',uk');
    plant = drss (12,12,4);
    plant.A = A;
    plant.B = B;
    plant.C = eye(12);
    plant.D = 0;
    plant.Ts = SimParams.dt;
    
    SimParams.sigma_max = sdash_rbf(SimParams.XLim',SimParams.rbf_c,SimParams.rbf_mu,SimParams.bw,SimParams.n2);
    MaxStateDisturbance = [1.5;1.5;1.5;0.5;0.5;0.5;1;1;1;0.5;0.5;0.5];
    SimParams.Ka_max = SimParams.sigma_max*(MaxStateDisturbance'*pinv(plant.B'));
    SimParams.Kmax = norm(SimParams.Ka_max);
    SimParams.uamax = SimParams.Ka_max'*SimParams.sigma_max;
    SimParams.wmax = 2*plant.B*(SimParams.Ka_max'*SimParams.sigma_max);
    w_vertex = combinations(SimParams.wmax');
    SimParams.W = Polyhedron(w_vertex);
    umax_prime = SimParams.UMax'-SimParams.uamax; % available control authority for nominal MPC, it should be positive
    Uc_prime_vertex = combinations(umax_prime'); 
    SimParams.Uc_prime = Polyhedron(Uc_prime_vertex);
end