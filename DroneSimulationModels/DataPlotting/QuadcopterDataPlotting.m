function QuadcopterDataPlotting(FileName, FileNameRaw)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                    %
% Version Name: QuadcopterDataPlotting      %
% Date: 10/21/2021                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plotting Setup
SimParams = SimulationParameters();
%load(FileName,'SimResult');
load(FileName);
time = 0:SimParams.dt:SimParams.Duration;

%% Plot the Quadcopter States
figure('Name','States')
subplot(2,3,1)
hold on
plot(time,SimResult.Nominal.RefTraj(1,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(1,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(1,:))
plot(time,SimResult.Adapt.StateTraj(1,:))
plot(time,SimResult.Deep.StateTraj(1,:))
grid on
xlabel('Time (s)')
ylabel('Position (m)')
title('Qruadrotor X position')

subplot(2,3,2)
hold on
plot(time,SimResult.Nominal.RefTraj(2,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(2,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(2,:))
plot(time,SimResult.Adapt.StateTraj(2,:))
plot(time,SimResult.Deep.StateTraj(2,:))
grid on
xlabel('Time (s)')
ylabel('Position (m)')
title('Qruadrotor Y position')

subplot(2,3,3)
hold on
plot(time,SimResult.Nominal.RefTraj(3,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(3,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(3,:))
plot(time,SimResult.Adapt.StateTraj(3,:))
plot(time,SimResult.Deep.StateTraj(3,:))
grid on
xlabel('Time (s)')
ylabel('Position (m)')
title('Qruadrotor Z position')

subplot(2,3,4)
hold on
plot(time,SimResult.Nominal.RefTraj(4,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(4,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(4,:))
plot(time,SimResult.Adapt.StateTraj(4,:))
plot(time,SimResult.Deep.StateTraj(4,:))
grid on
xlabel('Time (s)')
ylabel('Angle (radians)')
title('Qruadrotor Pitch Angle')

subplot(2,3,5)
hold on
plot(time,SimResult.Nominal.RefTraj(5,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(5,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(5,:))
plot(time,SimResult.Adapt.StateTraj(5,:))
plot(time,SimResult.Deep.StateTraj(5,:))
grid on
xlabel('Time (s)')
ylabel('Angle (radians)')
title('Qruadrotor Roll Angle')

subplot(2,3,6)
hold on
plot(time,SimResult.Nominal.RefTraj(6,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Nominal.StateTraj(6,1:end-SimParams.PredictionHorizon))
plot(time,SimResult.Tube.StateTraj(6,:))
plot(time,SimResult.Adapt.StateTraj(6,:))
plot(time,SimResult.Deep.StateTraj(6,:))
grid on
xlabel('Time (s)')
ylabel('Angle (radians)')
legend('Reference Trajectory','Nominal Trajectory','Tube MPC Trajectory','Adaptive MPC Trajectory','Deep MPC Trajectory','Location','southeast')
title('Qruadrotor Psi angle')
saveas(gcf,[FileNameRaw,'_States.fig']);

%% Figure-8 Tracing Plot
figure('Name','Trajectory Tracing')
hold on
plot(SimResult.Nominal.RefTraj(1,1:end-SimParams.PredictionHorizon),SimResult.Nominal.RefTraj(2,1:end-SimParams.PredictionHorizon))
plot(SimResult.Nominal.StateTraj(1,1:end-SimParams.PredictionHorizon),SimResult.Nominal.StateTraj(2,1:end-SimParams.PredictionHorizon))
plot(SimResult.Tube.StateTraj(1,:),SimResult.Tube.StateTraj(2,:))
plot(SimResult.Adapt.StateTraj(1,:),SimResult.Adapt.StateTraj(2,:))
plot(SimResult.Deep.StateTraj(1,:),SimResult.Deep.StateTraj(2,:))
grid on
xlabel('X Position')
ylabel('Y Position')
legend('Reference Trajectory','Nominal Trajectory','Tube MPC Trajectory','Adapt MPC Trajectory','Deep MPC Trajectory','Location','southeast')
title('Trajectory Tracing')
saveas(gcf,[FileNameRaw,'_TrajectoryTracing.fig'])

%% Control Inputs
figure('Name','Control Inputs')
subplot(2,2,1)
hold on
plot(time,SimResult.Tube.ControlInput(1,:))
plot(time,SimResult.Adapt.ControlInput(1,:))
plot(time,SimResult.Deep.ControlInput(1,:))
%     ylim([-0.5,30])
grid on
xlabel('time')
title('Torque Pitch Input')

subplot(2,2,2)
hold on
plot(time,SimResult.Tube.ControlInput(2,:))
plot(time,SimResult.Adapt.ControlInput(2,:))
plot(time,SimResult.Deep.ControlInput(2,:))
%     ylim([-0.5,30])
grid on
xlabel('time')
title('Torque Roll Input')

subplot(2,2,3)
hold on
plot(time,SimResult.Tube.ControlInput(3,:))
plot(time,SimResult.Adapt.ControlInput(3,:))
plot(time,SimResult.Deep.ControlInput(3,:))
%     ylim([-0.5,30])
grid on
xlabel('time')
title('Torque Yaw Input')

subplot(2,2,4)
hold on
plot(time,SimResult.Tube.ControlInput(4,:))
plot(time,SimResult.Adapt.ControlInput(4,:))
plot(time,SimResult.Deep.ControlInput(4,:))
%     ylim([-0.5,30])
grid on
xlabel('time')
title('Thrust Input')
legend('Tube MPC','Adapt MPC','Deep MPC','Location','southeast')
saveas(gcf,[FileNameRaw,'_ControlInputs.fig'])
clear all;
end