function StdDeviation(FileName,FileNameRaw)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan          %
% Version Name: StdDeviation      %
% Last Modified: 10/29/2021       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Setup Parameters and Load Data
    load(FileName,'SimResult');
    SimParams = SimulationParameters();
    
    %% Run Standard Deviation Calculations
    [tube1, adapt1, deep1] = StdDeviationCalculator(1,(SimParams.Duration/SimParams.dt)*1/5,SimResult);
    [tube2, adapt2, deep2] = StdDeviationCalculator((SimParams.Duration/SimParams.dt)*1/5+1,(SimParams.Duration/SimParams.dt)*2/5,SimResult);
    [tube3, adapt3, deep3] = StdDeviationCalculator((SimParams.Duration/SimParams.dt)*2/5+1,(SimParams.Duration/SimParams.dt)*3/5,SimResult);
    [tube4, adapt4, deep4] = StdDeviationCalculator((SimParams.Duration/SimParams.dt)*3/5+1,(SimParams.Duration/SimParams.dt)*4/5,SimResult);
    [tube5, adapt5, deep5] = StdDeviationCalculator((SimParams.Duration/SimParams.dt)*4/5+1,(SimParams.Duration/SimParams.dt)*5/5,SimResult);
    [tube6, adapt6, deep6] = StdDeviationCalculator(1,(SimParams.Duration/SimParams.dt)*5/5,SimResult);

    %% Create tables and save them to excel files for Altitude and R-Distance
    r_distance = [tube1(1),tube2(1),tube3(1),tube4(1),tube5(1),tube6(1);
        adapt1(1),adapt2(1),adapt3(1),adapt4(1),adapt5(1),adapt6(1);
        deep1(1),deep2(1),deep3(1),deep4(1),deep5(1),deep6(1)];
    r_VarNames = {'Takeoff','Iter1','Iter2','Iter3','Iter4','Average'};
    r_rowNames = {'Tube MPC','Adaptive MPC', 'Deep MPC'};
    r_Table = table(r_distance(:,1),r_distance(:,2),r_distance(:,3),r_distance(:,4),r_distance(:,5),r_distance(:,6), 'VariableNames',r_VarNames,'RowNames',r_rowNames);
    disp('R-Distance Error');
    disp(r_Table);
    write(r_Table,[FileNameRaw,'_RDistanceError','.xlsm']);

    x_distance = [tube1(2),tube2(2),tube3(2),tube4(2),tube5(2),tube6(2);
        adapt1(2),adapt2(2),adapt3(2),adapt4(2),adapt5(2),adapt6(2);
        deep1(2),deep2(2),deep3(2),deep4(2),deep5(2),deep6(2)];
    x_VarNames = {'Takeoff','Iter1','Iter2','Iter3','Iter4','Average'};
    x_rowNames = {'Tube MPC','Adaptive MPC', 'Deep MPC'};
    x_Table = table(x_distance(:,1),x_distance(:,2),x_distance(:,3),x_distance(:,4),x_distance(:,5),x_distance(:,6), 'VariableNames',x_VarNames,'RowNames',x_rowNames);
    disp('X Position Error');
    disp(x_Table);
    write(x_Table,[FileNameRaw,'_XPositionError','.xlsm']);
    
    y_distance = [tube1(3),tube2(3),tube3(3),tube4(3),tube5(3),tube6(3);
        adapt1(3),adapt2(3),adapt3(2),adapt4(3),adapt5(3),adapt6(3);
        deep1(3),deep2(3),deep3(3),deep4(3),deep5(3),deep6(3)];
    y_VarNames = {'Takeoff','Iter1','Iter2','Iter3','Iter4','Average'};
    y_rowNames = {'Tube MPC','Adaptive MPC', 'Deep MPC'};
    y_Table = table(y_distance(:,1),y_distance(:,2),y_distance(:,3),y_distance(:,4),y_distance(:,5),y_distance(:,6), 'VariableNames',y_VarNames,'RowNames',y_rowNames);
    disp('Y Position Error');
    disp(y_Table);
    write(y_Table,[FileNameRaw,'_YPositionError','.xlsm']);
    
    z_distance = [tube1(4),tube2(4),tube3(4),tube4(4),tube5(4),tube6(4);
        adapt1(4),adapt2(4),adapt3(4),adapt4(4),adapt5(4),adapt6(4);
        deep1(4),deep2(4),deep3(4),deep4(4),deep5(4),deep6(4)];
    z_VarNames = {'Takeoff','Iter1','Iter2','Iter3','Iter4','Average'};
    z_rowNames = {'Tube MPC','Adaptive MPC', 'Deep MPC'};
    z_Table = table(z_distance(:,1),z_distance(:,2),z_distance(:,3),z_distance(:,4),z_distance(:,5),z_distance(:,6), 'VariableNames',z_VarNames,'RowNames',z_rowNames);
    disp('Altitude Error');
    disp(z_Table);
    write(z_Table,[FileNameRaw,'_AltitudeError','.xlsm']);
end