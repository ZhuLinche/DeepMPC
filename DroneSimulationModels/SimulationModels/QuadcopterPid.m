function [ThrustRef, TrajError, SimParams] = QuadcopterPid(SimParams, RefTraj, EstimTrajPrev)
    XPrev = EstimTrajPrev(1);
    YPrev = EstimTrajPrev(2);
    ZPrev = EstimTrajPrev(3);
    PhiPrev = EstimTrajPrev(4);
    ThetaPrev = EstimTrajPrev(5);
    PsiPrev = EstimTrajPrev(6);
    UPrev = EstimTrajPrev(7);
    VPrev = EstimTrajPrev(8);
    WPrev = EstimTrajPrev(9);
    PPrev = EstimTrajPrev(10);
    QPrev = EstimTrajPrev(11);
    RPrev = EstimTrajPrev(12);
    
    XRefTraj = RefTraj(1);
    YRefTraj = RefTraj(2);
    ZRefTraj = RefTraj(3);
    YawRefTraj = RefTraj(6);
    
    %%Outer Loop Control
    %Converts X,Y,Z trajectory inputs to Roll, Pitch, Yaw trajectory inputs
    ErrorX = XRefTraj - XPrev;
    ErrorY = YRefTraj - YPrev;
    
    SimParams.IntegrationValX = IntegralFunction(SimParams.IntegrationValX, ErrorX, SimParams);
    SimParams.IntegrationValY = IntegralFunction(SimParams.IntegrationValY, ErrorY, SimParams);
    
    PIDX = PIDControl(SimParams.PotentialX, SimParams.IntegralX, SimParams.DerivativeX, ErrorX, SimParams.IntegrationValX, UPrev);
    PIDY = PIDControl(SimParams.PotentialY, SimParams.IntegralY, SimParams.DerivativeY, ErrorY, SimParams.IntegrationValY, VPrev);
    
    PitchRef = PIDX*cos(PsiPrev)+PIDY*sin(PsiPrev);
    RollRef = PIDX*sin(PsiPrev)-PIDY*cos(PsiPrev);
    
    %%Thrust Control
    %Determines thrust input needed to maintain altitude
    ErrorZ = ZRefTraj - ZPrev;
    ThrustForcePID = PIDControl(SimParams.PotentialZ, 0, SimParams.DerivativeZ, ErrorZ, 0, WPrev);
    
    %%Innter Loop Control
    %Converts Pitch, Roll, Yaw trajectory inputs to control commands for
    %the drone
    ErrorYaw = YawRefTraj - PsiPrev;
    ErrorPitch = PitchRef - ThetaPrev;
    ErrorRoll = RollRef - PhiPrev;
    
    SimParams.IntegrationValPitch = IntegralFunction(SimParams.IntegrationValPitch, ErrorPitch, SimParams);
    SimParams.IntegrationValRoll = IntegralFunction(SimParams.IntegrationValRoll, ErrorRoll, SimParams);
    
    TorqueYaw = PIDControl(SimParams.PotentialYaw, 0, SimParams.DerivativeYaw, ErrorYaw, 0, RPrev);
    TorquePitch = PIDControl(SimParams.PotentialPitch, SimParams.IntegralPitch, SimParams.DerivativePitch, ErrorPitch, SimParams.IntegrationValPitch, QPrev);
    TorqueRoll = PIDControl(SimParams.PotentialRoll, SimParams.IntegralRoll, SimParams.DerivativeRoll, ErrorRoll, SimParams.IntegrationValRoll, PPrev);
    
    %%Output Formating
    ThrustRef = [TorqueRoll, TorquePitch, TorqueYaw, ThrustForcePID];
    ThrustRef = SimParams.ThrustGain*ThrustRef;
    TrajError = [ErrorX, ErrorY, ErrorZ, ErrorRoll, ErrorPitch, ErrorYaw];
end
