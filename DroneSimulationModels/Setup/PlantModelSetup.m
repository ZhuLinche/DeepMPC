%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                          %
% Version Name: PlantModelSetup                   %
% Last Modified: 12/06/2021                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plantmodels = PlantModelSetup(SimParams)
Ts = SimParams.dt;
switch SimParams.DisturbanceCase
    case "Mass Change"
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [15.3568 15.3568 15.3568 15.3568];
    case "High Mass COM Shift"   
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [15.3568 15.3568 15.3568 15.3568];
    otherwise    
        xk = [0 0 1 0 0 0 0 0 0 0 0 0];
        uk = [14.4 14.4 14.4 14.4];
end

plantmodels = struct();

[A,B] = QuadrotorStateJacobianFcn(xk',uk');
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;
plantmodels.NominalPlantModel = plant;

[A,B] = QuadrotorStateJacobianFcnBladeDamage(xk',uk');
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;
plantmodels.BladeDamagePlantModel = plant;

[A,B] = QuadrotorStateJacobianFcnCOMShift(xk',uk');
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;
plantmodels.COMShiftPlantModel = plant;

[A,B] = QuadrotorStateJacobianFcnMassCapacity(xk',uk');
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;
plantmodels.HighMassPlantModel = plant;

[A,B] = QuadrotorStateJacobianFcnHarshCOMShiftWeightCapacity(xk',uk');
plant = drss (12,12,4);
plant.A = A;
plant.B = B;
plant.C = eye(12);
plant.D = 0;
plant.Ts = Ts;
plantmodels.HighMassPlantModelCOMShift = plant;