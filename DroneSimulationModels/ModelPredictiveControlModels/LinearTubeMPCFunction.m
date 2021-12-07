function TubeMPC = TubeMPCFunction(TubeMPC,RefTraj,plant,MPCobj,xmpc,MPCopt, Noise, SimParams)
    %% MPC Sim and Propogation
    % This part of the MPC code takes the trajectory, previous output and setup
    % parameters to determine the next control input. It then propogates the
    % system and stores the data
    
    EstimTrajPrev = TubeMPC.EstimTraj;
    y = plant.C*EstimTrajPrev; % calculate plant output
    
    ThrustRef = mpcmove(MPCobj,xmpc,y,RefTraj',[],MPCopt); % calculate optimal mpc move
    switch SimParams.DisturbanceCase
        case "Damaged Blade - No Wind Bias"
            ThrustRef = ThrustRef*0.5;
    end
    EstimTraj = plant.A*(EstimTrajPrev+Noise.StateNoise)+plant.B*(ThrustRef+Noise.ControlNoise); % update plant state

    %% Store Results
    TubeMPC = struct();
    TubeMPC.EstimTraj = EstimTraj;
    TubeMPC.ThrustRef = ThrustRef;
end