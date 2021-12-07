function NominalMPC = LinearNominalMPCFunction(NominalMPC,RefTraj,plant,MPCobj,xmpc)
    %% MPC Sim and Propogation
    % This part of the MPC code takes the trajectory, previous output and setup
    % parameters to determine the next control input. It then propogates the
    % system and stores the data

    EstimTrajPrev = NominalMPC.EstimTraj;
    y = plant.C*EstimTrajPrev; % calculate plant output
    
    ThrustRef = mpcmove(MPCobj,xmpc,y,RefTraj'); % calculate optimal mpc move
    EstimTraj = plant.A*EstimTrajPrev+plant.B*ThrustRef; % update plant state
    
    %% Store Results
    NominalMPC = struct();
    NominalMPC.EstimTraj = EstimTraj;
    NominalMPC.ThrustRef = ThrustRef;
end