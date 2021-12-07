function [EstimTraj, VelocityBody] = QuadcopterSim(SimParams,EstimTrajPrev, ThrustRef, VelocityBody)
    %Function Input to Function Variable Formatting
    XPrev = EstimTrajPrev(1);
    YPrev = EstimTrajPrev(2);
    ZPrev = EstimTrajPrev(3);
    PhiPrev = EstimTrajPrev(4);
    ThetaPrev = EstimTrajPrev(5);
    PsiPrev = EstimTrajPrev(6);
    UPrev = VelocityBody(1);
    VPrev = VelocityBody(2);
    WPrev = VelocityBody(3);
    PPrev = EstimTrajPrev(10);
    QPrev = EstimTrajPrev(11);
    RPrev = EstimTrajPrev(12);
    
    TorqueRoll = ThrustRef(1);
    TorquePitch = ThrustRef(2);
    TorqueYaw = ThrustRef(3);
    Thrust = ThrustRef(4);
    
    %Simulate Next Position
    RBody2Vehicle = [cos(ThetaPrev)*cos(PsiPrev), sin(PhiPrev)*sin(ThetaPrev)*cos(PsiPrev)-cos(PhiPrev)*sin(PsiPrev), cos(PhiPrev)*sin(ThetaPrev)*cos(PsiPrev)+sin(PhiPrev)*sin(PsiPrev);...
        cos(ThetaPrev)*sin(PsiPrev), sin(PhiPrev)*sin(ThetaPrev)*sin(PsiPrev)+cos(PhiPrev)*cos(PsiPrev), cos(PhiPrev)*sin(ThetaPrev)*sin(PsiPrev)-sin(PhiPrev)*cos(PsiPrev);...
        -sin(ThetaPrev), sin(PhiPrev)*cos(ThetaPrev), cos(PhiPrev)*cos(ThetaPrev)];
    X = XPrev + RBody2Vehicle(1,:)*[UPrev; VPrev; WPrev].*SimParams.dt;
    Y = YPrev + RBody2Vehicle(2,:)*[UPrev; VPrev; WPrev].*SimParams.dt;
    Z = ZPrev + RBody2Vehicle(3,:)*[UPrev; VPrev; WPrev].*SimParams.dt;
    
    %Simulate Next Oritentation
    Phi = PhiPrev + (PPrev+QPrev*sin(PhiPrev)*tan(ThetaPrev)+RPrev*cos(PhiPrev)*tan(ThetaPrev))*SimParams.dt;
    Theta = ThetaPrev + (QPrev*cos(PhiPrev) - RPrev*sin(PhiPrev))*SimParams.dt;
    Psi = PsiPrev + ((QPrev*sin(PhiPrev) + RPrev*cos(PhiPrev))/cos(ThetaPrev))*SimParams.dt;
    
    %Simulate Next Velocity
    u = UPrev + (RPrev*VPrev-QPrev*WPrev - SimParams.g*sin(ThetaPrev))*SimParams.dt;
    v = VPrev + (PPrev*WPrev-RPrev*UPrev + SimParams.g*cos(ThetaPrev)*sin(PhiPrev))*SimParams.dt;
    w = WPrev + (QPrev*UPrev-PPrev*VPrev + SimParams.g*cos(ThetaPrev)*cos(PhiPrev)+Thrust/SimParams.Mass)*SimParams.dt;
    velocity = RBody2Vehicle*[u;v;w];

    %Simulate Next Angular Rates
    p = PPrev + (((SimParams.Jy-SimParams.Jz)/SimParams.Jx)*QPrev*RPrev + TorqueRoll/SimParams.Jx)*SimParams.dt;
    q = QPrev + (((SimParams.Jz-SimParams.Jx)/SimParams.Jy)*PPrev*RPrev + TorquePitch/SimParams.Jy)*SimParams.dt;
    r = RPrev + (((SimParams.Jx-SimParams.Jy)/SimParams.Jz)*PPrev*QPrev + TorqueYaw/SimParams.Jz)*SimParams.dt;
    
    %Output Formating
    EstimTraj = [X; Y; Z; Phi; Theta; Psi; velocity(1); velocity(2); velocity(3); p; q; r];
    VelocityBody = [u, v, w];
end