%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: ControlDisturbance                %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Generates a Control Noise for a 4 Input based System

function ControlNoise = ControlDisturbance(SimTime, ControlGain,DronePosition)
    if SimTime > 15 %&& SimTime < 20
        DistU1 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
        DistU2 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
        DistU3 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
        DistU4 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%     elseif SimTime > 40 && SimTime < 45
%         DistU1 = -ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU2 = -0.98*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU3 = -0.99*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU4 = -ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%     elseif SimTime > 50 && SimTime < 55
%         DistU1 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU2 = 0.98*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU3 = 0.99*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU4 = ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%     elseif SimTime > 60 && SimTime < 65
%         DistU1 = -ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU2 = -0.98*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU3 = -0.99*ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
%         DistU4 = -ControlGain*min(max(DronePosition*abs(DronePosition),-0.5),0.5);
    else
        DistU1 = 0;
        DistU2 = 0;
        DistU3 = 0;
        DistU4 = 0;
    end
    ControlNoise = [DistU1;DistU2;DistU3;DistU4];
end