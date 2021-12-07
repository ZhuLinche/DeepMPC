%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: WindDisturbance                   %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Generates a Wind Disturbance for a 12 output system
function Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain)
    if SimTime>30 && SimTime <55 %|| (SimTime>35 && SimTime <45)
        Vm = 1;
        dm = 3;
        rho = 1;
        drag = zeros(3,1);
        % generates wind force on the drone which varies according to
        % position of drone (use this directly in simulating wind)
        state = [DronePosition(1);DronePosition(2);DronePosition(3)]; % 3xn matrix, (x;y;z)
        [m,n] = size(state);
        for i = 1:m
            if norm(state(i)) > dm % at i=1, implies at first time instant,we check the norm(x,y,z). If greater, than 3, than wind force [3*0.25;3*0.25;3*0.25].
                wind = 0.25*Vm;
            elseif norm(state(i)) >=0 && norm(state(i)) <= dm
                wind = WindGain.X*Vm*(1 + cos(pi*state(i)/dm));
            else
                wind = 0*obj.Vm;
            end
            drag(i) = 0.5*rho*wind.^2;
        end
		DistX = 0;
        DistY = 0;
        DistZ = 0;
        DistPhi = 0;
        DistTheta = 0;
        DistPsi = 0;
        DistU = drag(1);
        DistV = drag(2);
        DistW = 0; %Z-direction experiences no wind
        DistP = 0;
        DistQ = 0;
        DistR = 0;
    else
        DistX = 0;
        DistY = 0;
        DistZ = 0;
        DistPhi = 0;
        DistTheta = 0;
        DistPsi = 0;
        DistU = 0;
        DistV = 0;
        DistW = 0;
        DistP = 0;
        DistQ = 0;
        DistR = 0;
    end
    Noise.StateNoise = [DistX;DistY;DistZ;DistPhi;DistTheta;DistPsi;DistU;DistV;DistW;DistP;DistQ;DistR];
end