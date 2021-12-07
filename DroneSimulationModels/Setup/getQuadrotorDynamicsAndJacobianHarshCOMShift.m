% This script defines a continuous-time nonlinear quadrotor model and
% generates a state function and its Jacobian function used by the
% nonlinear MPC controller in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Create symbolix variables for states, MVs and parameters
syms xt(t) yt(t) zt(t) phit(t) thetat(t) psit(t) xdott(t) ydott(t)...
    zdott(t) phidott(t) thetadott(t) psidott(t) 
syms u1 u2 u3 u4 Ixx Iyy Izz k l m b g
syms x y z phi theta psi xdot ydot zdot phidot thetadot psidot

% phi: roll angle 
% theta: pitch angle 
% psi: yaw angle 
% ui:squared angular velocity of rotor i
% g: gravity
% b: drag constant
% k: lift constant
% l: distance between rotor and com
% Iii: diagonal elements of inertia matrix

% Set values for dynamics parameters
IxxVal = 0.000058286*1.5; 
IyyVal = 0.000071691*0.5;
IzzVal = 0.0001;
mVal = 0.067;
kVal = 0.0107;
lVal = 0.0624;
bVal = 0.78264*10^-3;
gVal = 9.81;

% IxxVal = 1.2; 
% IyyVal = 1.2;
% IzzVal = 2.3;
% kVal = 1;
% lVal = 0.25;
% mVal = 2;
% bVal = 0.2;
% gVal = 9.81;

paramValues = [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal];

% Group symbolic variables
statet = {xt(t) yt(t) zt(t) phit(t) thetat(t) psit(t) xdott(t) ...
    ydott(t) zdott(t) phidott(t) thetadott(t) psidott(t)};
state = {x y z phi theta psi xdot ydot zdot phidot thetadot psidot};
state_diff = {diff(xt(t),t), diff(yt(t),t), diff(zt(t),t), ...
    diff(phit(t),t), diff(thetat(t),t), diff(psit(t),t)};
state_dot = {xdot ydot zdot phidot thetadot psidot};

% Transformation matrix for angular velocities from inertial frame 
% to body frame
W = [1, 0, -sin(thetat);
    0, cos(phit), cos(thetat)*sin(phit);
    0, -sin(phit), cos(thetat)*cos(phit)];

%R-ZYX Euler
Rz = [cos(psit), -sin(psit), 0;
    sin(psit), cos(psit), 0;
    0, 0, 1];
Ry = [cos(thetat), 0, sin(thetat);
    0, 1, 0;
    -sin(thetat), 0, cos(thetat)];
Rx = [1, 0, 0;
    0, cos(phit), -sin(phit);
    0, sin(phit), cos(phit)];

% Rotation matrix from body frame to inertial frame
R = Rz*Ry*Rx;

% Jacobian (relates body frame to inertial frame velocities)
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;

% Coriolis forces
dJ_dt = diff(J);
dJ_dt = subs(dJ_dt,[state_diff statet],[state_dot state]);
h_dot_J = [phidott(t), thetadott(t), psidott(t)]*J;
h_dot_J = subs(h_dot_J,[state_diff statet],[state_dot state]);
grad_temp_h = jacobian(h_dot_J,[phi theta psi]);
C = dJ_dt - 1/2*grad_temp_h;

% Torques in the direction of phi, theta, psi
tau_beta = [l*k*(-u2 + u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];

% Total thrust
T = k*(u1+u2+u3+u4);

% Dynamics
f(1) = xdott;
f(2) = ydott;
f(3) = zdott;
f(4) = phidott;
f(5) = thetadott;
f(6) = psidott;

% Equations for COM configuration
f(7:9) = -g*[0;0;1] + R*[0;0;T]/m;

% Euler Lagrange equations for angular dynamics
f(10:12) = inv(J)*(tau_beta - C*[phidott(t); thetadott(t); psidott(t)]);

% Replace parameters and drop time dependence
f = subs(f, [Ixx Iyy Izz k l m b g], paramValues);
f = subs(f,statet,state);
f = simplify(f);

% Calculate linearization
A = jacobian(f,[state{:}]);
control = [u1, u2, u3, u4];
B = jacobian(f,control);

% Create QuadrotorStateFcn.m
matlabFunction(transpose(f),'File','QuadrotorStateFcnHarshCOMShiftWeightCapacity',...
    'Vars',{transpose([state{:}]),transpose(control)});
% Create QuadrotorStateJacobianFcn.m 
matlabFunction(A, B,'File','QuadrotorStateJacobianFcnHarshCOMShiftWeightCapacity',...
    'Vars',{transpose([state{:}]),transpose(control)});

%Clear symbolic variables
clear
