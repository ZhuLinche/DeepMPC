%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                    %
% Version Name: QuadDataProcessing          %
% Date: 10/21/2021                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear current Workspace
clear all;
close all;
clearvars -global;

%% Add Needed Folders to Project Directory that will be needed for creating plots and processed data sets
currentFolder = pwd;
addpath([currentFolder, '\Results']);
addpath([currentFolder, '\ProcessedResults']);
addpath([currentFolder, '\DataPlotting']);

File = uigetfile('*.mat');
FileName = extractBefore(File,'.mat');
FileName = extractAfter(FileName,'_');
mkdir([currentFolder, '\ProcessedResults'], FileName);
addpath([[currentFolder, '\ProcessedResults'], FileName]);
ResultName = [FileName,'_Full'];
DestinationName = FileName;
load(File);
SimEnd = length(SimResult.Tube.StateTraj);
resultperiod = [1, SimParams.Duration/SimParams.dt];

%% These functions create plots for tracking Trajectory of the system for the Manuver, Position, Orientation and finding the deviation from the trajectory
TrajectoryPlotting(SimResult, resultperiod, ResultName, DestinationName);
PositionTracking(SimResult,SimParams, resultperiod, ResultName, DestinationName);
OrientationTracking(SimResult,SimParams, resultperiod, ResultName, DestinationName);
rsquaredata = RSquared(SimResult, SimParams, resultperiod);
msedata = MSE(SimResult, SimParams, resultperiod);
m1_data = DisplacementError(SimResult, SimParams, resultperiod, ResultName,DestinationName);
MSERSquaredData(rsquaredata, msedata, m1_data, ResultName, DestinationName);
disp('Data Processing Completed and Stored');

function TrajectoryPlotting(SimResult, resultperiod, ResultName, DestinationName)
    YMaxLim = max([max(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)))]);
    XMaxLim = max([max(SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)))]);
    YMinLim = min([min(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)))]);
    XMinLim = min([min(SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)))]);
    XLim = ceil(max([XMaxLim,abs(XMinLim)]))+.25;
    YLim = ceil(max([YMaxLim,abs(YMinLim)]))+.25;

    figure('Name','Trajectory')
    sgtitle('XY Trajectory')
    subplot(1,3,1)
    hold on
    plot(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Nominal.RefTraj(1,resultperiod(1):resultperiod(2)))
    plot(SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2)),'r')
    grid on
    xlabel('X')
    ylabel('Y')
    ylim([-0.25,2.25])
    xlim([-YLim,YLim])
    legend('Nominal','Tube MPC','location','northwest')
    title('Tube MPC')

    subplot(1,3,2)
    hold on
    plot(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Nominal.RefTraj(1,resultperiod(1):resultperiod(2)))
    plot(SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2)),'b')
    grid on
    xlabel('X')
    ylabel('Y')
    ylim([-0.25,2.25])
    xlim([-YLim,YLim])
    legend('Nominal','Adapt MPC','location','northwest')
    title('Adaptive MPC')

    subplot(1,3,3)
    hold on
    plot(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Nominal.RefTraj(1,resultperiod(1):resultperiod(2)))
    plot(SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)),SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)),'g')
    grid on
    xlabel('X')
    ylabel('Y')
    ylim([-0.25,2.25])
    xlim([-YLim,YLim])
    legend('Nominal','Deep MPC','location','northwest')
    title('Deep MPC')
    saveas(gcf,[[ResultName,'_Trajectory'],'.fig'])
    movefile([[ResultName,'_Trajectory'],'.fig'],[pwd,'\ProcessedResults\',DestinationName]);
    close Figure1: Trajectory
end
function PositionTracking(SimResult,SimParams, resultperiod, ResultName, DestinationName)
    YMaxLim = max([max(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)))]);
    XMaxLim = max([max(SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)))]);
    ZMaxLim = max([max(SimResult.Nominal.StateTraj(3,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(3,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(3,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(3,resultperiod(1):resultperiod(2)))]);
    YMinLim = min([min(SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)))]);
    XMinLim = min([min(SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)))]);
    ZMinLim = min([min(SimResult.Nominal.StateTraj(3,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(3,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(3,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(3,resultperiod(1):resultperiod(2)))]);
    
    XLim = ceil(max([XMaxLim,abs(XMinLim)]))+.25;
    YLim = ceil(max([YMaxLim,abs(YMinLim)]))+.25;
    ZLim = ceil(max([ZMaxLim,abs(ZMinLim)]))+.25;
    switch SimParams.TrajectoryCase
        case 'Hover'
            time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
            figure('Name','Position')
            sgtitle('Position Trajectory Tracking')
            subplot(3,1,1)
            hold on
            plot(time,SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('X')
            ylim([-XLim,XLim])
            legend('Nominal','Tube MPC','Adapt MPC','Deep MPC','location','northwest')

            subplot(3,1,2)
            hold on
            plot(time,SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Y')
            ylim([-YLim,YLim])

            subplot(3,1,3)
            hold on
            plot(time,SimResult.Nominal.StateTraj(3,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(3,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(3,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(3,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Z')
            ylim([-ZLim,0.05])

        otherwise
            time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
            figure('Name','Position')
            sgtitle('Position Trajectory Tracking')
            subplot(3,1,1)
            hold on
            plot(time,SimResult.Nominal.StateTraj(1,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(1,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(1,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(1,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('X')
            ylim([-XLim,XLim])
            legend('Nominal','Tube MPC','Adapt MPC','Deep MPC','location','northwest')

            subplot(3,1,2)
            hold on
            plot(time,SimResult.Nominal.StateTraj(2,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(2,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(2,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(2,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Y')
            ylim([-YLim,YLim])

            subplot(3,1,3)
            hold on
            plot(time,SimResult.Nominal.StateTraj(3,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(3,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(3,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(3,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Z')
            ylim([-ZLim,0.05])
    end
    saveas(gcf,[[ResultName,'_Position'],'.fig'])
    movefile([[ResultName,'_Position'],'.fig'],[pwd, '\ProcessedResults\', DestinationName]);
    close Figure1: Position;
end
function OrientationTracking(SimResult,SimParams, resultperiod, ResultName, DestinationName)
    PMaxLim = max([max(SimResult.Nominal.StateTraj(4,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(4,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(4,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(4,resultperiod(1):resultperiod(2)))]);
    RMaxLim = max([max(SimResult.Nominal.StateTraj(5,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(5,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(5,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(5,resultperiod(1):resultperiod(2)))]);
    YMaxLim = max([max(SimResult.Nominal.StateTraj(6,resultperiod(1):resultperiod(2))),max(SimResult.Tube.StateTraj(6,resultperiod(1):resultperiod(2))),max(SimResult.Adapt.StateTraj(6,resultperiod(1):resultperiod(2))),max(SimResult.Deep.StateTraj(6,resultperiod(1):resultperiod(2)))]);
    PMinLim = min([min(SimResult.Nominal.StateTraj(4,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(4,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(4,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(4,resultperiod(1):resultperiod(2)))]);
    RMinLim = min([min(SimResult.Nominal.StateTraj(5,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(5,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(5,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(5,resultperiod(1):resultperiod(2)))]);
    YMinLim = min([min(SimResult.Nominal.StateTraj(6,resultperiod(1):resultperiod(2))),min(SimResult.Tube.StateTraj(6,resultperiod(1):resultperiod(2))),min(SimResult.Adapt.StateTraj(6,resultperiod(1):resultperiod(2))),min(SimResult.Deep.StateTraj(6,resultperiod(1):resultperiod(2)))]);
    
    PLim = ceil(max([PMaxLim,abs(PMinLim)]))+.25;
    RLim = ceil(max([RMaxLim,abs(RMinLim)]))+.25;
    YLim = ceil(max([YMaxLim,abs(YMinLim)]))+.25;
    switch SimParams.TrajectoryCase
        case 'Hover'
            time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
            figure('Name','Orientation')
            sgtitle('Orientation Trajectory Tracking')
            subplot(2,1,1)
            hold on
            plot(time,SimResult.Nominal.StateTraj(4,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(4,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(4,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(4,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Roll')
            ylim([-PLim,PLim])
            legend('Nominal','Tube MPC','Adapt MPC','Deep MPC','location','northwest')

            subplot(2,1,2)
            hold on
            plot(time,SimResult.Nominal.StateTraj(5,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(5,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(5,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(5,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Pitch')
            ylim([-RLim,RLim])
            
        otherwise
            time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
            figure('Name','Orientation')
            sgtitle('Orientation Trajectory Tracking')
            subplot(2,1,1)
            hold on
            plot(time,SimResult.Nominal.StateTraj(4,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(4,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(4,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(4,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Roll')
            ylim([-PLim,PLim])
            legend('Nominal','Tube MPC','Adapt MPC','Deep MPC','location','northwest')

            subplot(2,1,2)
            hold on
            plot(time,SimResult.Nominal.StateTraj(5,resultperiod(1):resultperiod(2)),'k')
            plot(time,SimResult.Tube.StateTraj(5,resultperiod(1):resultperiod(2)),'r')
            plot(time,SimResult.Adapt.StateTraj(5,resultperiod(1):resultperiod(2)),'b')
            plot(time,SimResult.Deep.StateTraj(5,resultperiod(1):resultperiod(2)),'g')
            grid on
            xlabel('Time')
            ylabel('Pitch')
            ylim([-RLim,RLim])

    end
    saveas(gcf,[[ResultName,'_Orientation'],'.fig'])
    movefile([[ResultName,'_Orientation'],'.fig'],[pwd, '\ProcessedResults\', DestinationName]);
    close all;
end
function rsquaredata = RSquared(SimResult, SimParams, resultperiod)
    labels = ["X-Position";"Y-Position";"Z-Position";"Roll-Oritnetation";"Pitch-Orientation"];
    rsquaredata = zeros(3,5);
    time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
    Nominal = SimResult.Nominal.StateTraj;
    Tube = SimResult.Tube.StateTraj;
    Adapt = SimResult.Adapt.StateTraj;
    Deep = SimResult.Deep.StateTraj;
    for i = 1:length(labels)
        TubeR = RSquaredCalculation(Nominal(i,resultperiod(1):resultperiod(2)),Tube(i,resultperiod(1):resultperiod(2)));
        AdaptR = RSquaredCalculation(Nominal(i,resultperiod(1):resultperiod(2)),Adapt(i,resultperiod(1):resultperiod(2)));
        DeepR = RSquaredCalculation(Nominal(i,resultperiod(1):resultperiod(2)),Deep(i,resultperiod(1):resultperiod(2)));
        rsquaredata(:,i) = [TubeR;AdaptR;DeepR];
    end
end
function rsquarevalue = RSquaredCalculation(x, y)
    [m,n] = size(x);
    numerator = n*sum(x.*y)-sum(x)*sum(y);
    denominator = sqrt((n*sum(x.^2)-sum(x)^2)*(n*sum(y.^2)-sum(y)^2));
    rsquarevalue = numerator/denominator;
end
function msedata = MSE(SimResult, SimParams, resultperiod)
    labels = ["X-Position";"Y-Position";"Radial";"Z-Position";"Roll-Oritnetation";"Pitch-Orientation"];
    msedata = zeros(3,5);
    Nominal = SimResult.Nominal.StateTraj;
    Tube = SimResult.Tube.StateTraj;
    Adapt = SimResult.Adapt.StateTraj;
    Deep = SimResult.Deep.StateTraj;
    for i = 1:length(labels)
        TubeR = MSECalculation(Nominal(i,resultperiod(1):resultperiod(2)),Tube(i,resultperiod(1):resultperiod(2)));
        AdaptR = MSECalculation(Nominal(i,resultperiod(1):resultperiod(2)),Adapt(i,resultperiod(1):resultperiod(2)));
        DeepR = MSECalculation(Nominal(i,resultperiod(1):resultperiod(2)),Deep(i,resultperiod(1):resultperiod(2)));
        msedata(:,i) = [TubeR;AdaptR;DeepR];
    end
end
function msevalue = MSECalculation(x, y)
    [m,n] = size(x);
    msevalue = (1/n)*sum((y-x).^2);
end
function m1_data = DisplacementError(SimResult, SimParams, resultperiod, ResultName,DestinationName);
    Nominal = SimResult.Nominal.StateTraj(:,resultperiod(1):resultperiod(2));
    Tube = SimResult.Tube.StateTraj(:,resultperiod(1):resultperiod(2));
    Adapt = SimResult.Adapt.StateTraj(:,resultperiod(1):resultperiod(2));
    Deep = SimResult.Deep.StateTraj(:,resultperiod(1):resultperiod(2));
    [m,n] = size(Nominal);
    d_tube = decalculation(Nominal,Tube);
    d_adapt = decalculation(Nominal,Adapt);
    d_deep = decalculation(Nominal,Deep);
    m1_data = [(1/n)*sum(d_tube(1,:)),(1/n)*sum(d_tube(2,:));(1/n)*sum(d_adapt(1,:)),(1/n)*sum(d_adapt(2,:));(1/n)*sum(d_deep(1,:)),(1/n)*sum(d_deep(2,:))];
    time = linspace(resultperiod(1),resultperiod(2)*SimParams.dt,((resultperiod(2)-resultperiod(1))+1));
    hold on
    title('XY Position Error')
    plot(time,d_tube(1,:),'r');
    plot(time,d_adapt(1,:),'b');
    plot(time,d_deep(1,:),'g');
    xlabel('Time')
    ylabel('Error')
    legend('Tube MPC','Adapt MPC','Deep MPC')
    saveas(gcf,[[ResultName,'_XYTrajectoryError'],'.fig'])
    movefile([[ResultName,'_XYTrajectoryError'],'.fig'],[pwd, '\ProcessedResults\', DestinationName]);
    close all;
    figure
    hold on
    title('Altitude Position Error')
    plot(time,d_tube(2,:),'r');
    plot(time,d_adapt(2,:),'b');
    plot(time,d_deep(2,:),'g');
    xlabel('Time')
    ylabel('Deviation')
    legend('Tube MPC','Adapt MPC','Deep MPC')
    saveas(gcf,[[ResultName,'_AltitudeTrajectoryError'],'.fig'])
    movefile([[ResultName,'_AltitudeTrajectoryError'],'.fig'],[pwd, '\ProcessedResults\', DestinationName]);    
    close all;
end
function d = decalculation(x,y)
    d(1,:) = sqrt((x(1,:)-y(1,:)).^2+(x(2,:)-y(2,:)).^2);
    d(2,:) = sqrt((x(3,:)-y(3,:)).^2);
end
function MSERSquaredData(rsquaredata, msedata, m1data, ResultName, DestinationName)
    VarNames = {'X','Y','Z','Roll','Pitch'};
    rowNames = {'Tube MPC','Adaptive MPC', 'Deep MPC'};
    rsquaredata_Table = table(rsquaredata(:,1),rsquaredata(:,2),rsquaredata(:,3),rsquaredata(:,4),rsquaredata(:,5), 'VariableNames',VarNames,'RowNames',rowNames);
    msedatadata_Table = table(msedata(:,1),msedata(:,2),msedata(:,3),msedata(:,4),msedata(:,5), 'VariableNames',VarNames,'RowNames',rowNames);
    m1data_Table = table(m1data(:,1),m1data(:,2),'VariableNames',{'XY','Z'},'RowNames',rowNames);
    writetable(rsquaredata_Table,[ResultName,'.xlsm'],'Sheet',1);
    writetable(msedatadata_Table,[ResultName,'.xlsm'],'Sheet',2);
    writetable(m1data_Table,[ResultName,'.xlsm'],'Sheet',3);
    movefile([ResultName,'.xlsm'],[pwd, '\ProcessedResults\', DestinationName]);

end