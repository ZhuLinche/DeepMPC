%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: DisturbanceMain                   %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Creates a disturbance case for a 12 Output, 4 Input based system. 
function Noise = DisturbanceMain(SimIteration, SimParams, Control, plantmodels, Uncertainty_data)
SimTime = SimIteration*SimParams.dt;
DronePosition = Control.EstimTraj;
WindGain = struct();
WindGain.Z = 0;
switch SimParams.DisturbanceCase
    case "Low Wind Bias"
        WindGain.X = 0.25;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Medium Wind Bias"
        WindGain.X = 0.625;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "High Wind Bias"
        WindGain.X = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Damaged Blade - No Wind Bias"
        WindGain.X = 0;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >30
            Noise.PlantModel = plantmodels.BladeDamagePlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Damaged Blade - Low Wind Bias"
        WindGain.X = 0.05;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.BladeDamagePlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Damaged Blade - Medium Wind Bias"
        WindGain.X = 0.1;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.BladeDamagePlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Damaged Blade - High Wind Bias"
        WindGain.X = 0.25*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >30
            Noise.PlantModel = plantmodels.BladeDamagePlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "COM Shift - No Wind Bias"
        WindGain.X = 0;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.COMShiftPlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "COM Shift - Low Wind Bias"
        WindGain.X = 0.05*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.COMShiftPlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "COM Shift - Medium Wind Bias"
        WindGain.X = 0.1*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.COMShiftPlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "COM Shift - High Wind Bias"
        WindGain.X = 0.25*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = [0;0;0;0];
        if (SimTime >30 && SimTime < 45) || (SimTime >55 && SimTime < 65) 
            Noise.PlantModel = plantmodels.COMShiftPlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Control Disturbance - No Wind Bias"
        WindGain.X = 0;
        ControlGain = 1.5*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(1));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Control Disturbance - Low Wind Bias"
        WindGain.X = 0.05;
        ControlGain = 10;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(1));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Control Disturbance - Medium Wind Bias"
        WindGain.X = 0.1;
        ControlGain = 10;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(1));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Control Disturbance - High Wind Bias"
        WindGain.X = 0.75;
        ControlGain = 1.5*Uncertainty_data(SimIteration);
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Low Wind Bias Hover"
        WindGain.X = 0;
        WindGain.Z = 0;
        ControlGain = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Medium Wind Bias Hover"
        WindGain.X = 0;
        WindGain.Z = 0;
        ControlGain = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "High Wind Bias Hover"
        WindGain.X = 0;
        WindGain.Z = 0;
        ControlGain = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        Noise.PlantModel = plantmodels.NominalPlantModel;
    case "Hover COM Shift High Wind Bias"
        WindGain.X = 0;
        WindGain.Z = 0;
        ControlGain = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        if (SimTime >30 && SimTime < 45) || (SimTime >55 && SimTime < 65) 
            Noise.PlantModel = plantmodels.COMShiftPlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Hover Blade Damage High Wind Bias"
        WindGain.X = 0;
        WindGain.Z = 0;
        ControlGain = 0.5;
        Noise = struct();
        Noise = WindDisturbance(Noise, DronePosition, SimTime, WindGain);
        Noise.ControlNoise = ControlDisturbance(SimTime, ControlGain, DronePosition(3));
        if SimTime >15
            Noise.PlantModel = plantmodels.BladeDamagePlantModel;
        else
            Noise.PlantModel = plantmodels.NominalPlantModel;
        end
    case "Mass Change"
        Noise = struct();
        Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0;0;0];
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
            Noise.PlantModel = plantmodels.NominalPlantModel;
        else
            Noise.PlantModel = plantmodels.HighMassPlantModel;
        end
    case "High Mass COM Shift"
        Noise = struct();
        Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0;0;0];
        Noise.ControlNoise = [0;0;0;0];
        if SimTime >15
        Noise.PlantModel = plantmodels.HighMassPlantModelCOMShift;
        else
            Noise.PlantModel = plantmodels.HighMassPlantModel;
        end
    otherwise
        Noise = struct();
        Noise.StateNoise = [0;0;0;0;0;0;0;0;0;0;0;0];
        Noise.ControlNoise = [0;0;0;0];
        Noise.PlantModel = plantmodels.NominalPlantModel;
end
end