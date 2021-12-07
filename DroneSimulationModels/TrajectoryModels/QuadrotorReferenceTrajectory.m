function [ xdesired ] = QuadrotorReferenceTrajectory(t, SimParams)
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

%#codegen
%Modified by Garrett Gowan

switch SimParams.TrajectoryCase
    case 'Figure8'
        xdesired = Figure8(t);
    case 'Circle'
        xdesired = Circle(t);
    case 'Hover'
        xdesired = Hover(t);
    otherwise
        xdesired = Hover(t);
end
end
    function xdesired = Figure8(t)
        if t(1)<10
           x = zeros(1,length(t));
           y = zeros(1,length(t));
           z = -ones(1,length(t)).*2;
        else
           x = 1*cos(2*pi*(t-5)/20);
           y = 1*sin(4*pi*(t-5)/20);
           z = -ones(1,length(t)).*2;
        end
        
        phi = zeros(1,length(t));
        theta = zeros(1,length(t));
        psi = zeros(1,length(t));
        xdot = zeros(1,length(t));
        ydot = zeros(1,length(t));
        zdot = zeros(1,length(t));
        phidot = zeros(1,length(t));
        thetadot = zeros(1,length(t));
        psidot = zeros(1,length(t));

        xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
    end

    function xdesired = Circle(t)
        if t(1)<10
           x = zeros(1,length(t));
           y = zeros(1,length(t));
           z = -ones(1,length(t)).*2;
        else
           x = 1*cos(t)+1;
           y = 1*sin((t));
           z = -ones(1,length(t)).*2;
        end

        phi = zeros(1,length(t));
        theta = zeros(1,length(t));
        psi = zeros(1,length(t));
        xdot = zeros(1,length(t));
        ydot = zeros(1,length(t));
        zdot = zeros(1,length(t));
        phidot = zeros(1,length(t));
        thetadot = zeros(1,length(t));
        psidot = zeros(1,length(t));

        xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
    end

    function xdesired = Hover(t)
        x = zeros(1,length(t));
        y = zeros(1,length(t));
        z = -ones(1,length(t)).*2;
        phi = zeros(1,length(t));
        theta = zeros(1,length(t));
        psi = zeros(1,length(t));
        xdot = zeros(1,length(t));
        ydot = zeros(1,length(t));
        zdot = zeros(1,length(t));
        phidot = zeros(1,length(t));
        thetadot = zeros(1,length(t));
        psidot = zeros(1,length(t));

        xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
    end
