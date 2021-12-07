function [Tube_std,Adapt_std,Deep_std] = StdDeviationCalculator(data1,data2,result)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Garrett Gowan                      %
% Version Name: StdDeviationCalculator        %
% Last Modified: 10/29/2021                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Find Std Deviation of R-Squared
    Nominal_R = sqrt(result.Nominal.StateTraj(1, data1:data2).^2+result.Nominal.StateTraj(2, data1:data2).^2);
    Tube_R = sqrt(result.Tube.StateTraj(1, data1:data2).^2+result.Tube.StateTraj(2, data1:data2).^2);
    Adapt_R = sqrt(result.Adapt.StateTraj(1, data1:data2).^2+result.Adapt.StateTraj(2, data1:data2).^2);
    Deep_R = sqrt(result.Deep.StateTraj(1, data1:data2).^2+result.Deep.StateTraj(2, data1:data2).^2);

    Tube_std(1,:) = sqrt(sum((Nominal_R-Tube_R).^2)/length(Nominal_R));
    Adapt_std(1,:) = sqrt(sum((Nominal_R-Adapt_R).^2)/length(Nominal_R));
    Deep_std(1,:) = sqrt(sum((Nominal_R-Deep_R).^2)/length(Nominal_R));

    %% Fin Std Deviation of Altitude and Velocities
    for i = 1:6
        Tube_std(i+1,:) = sqrt(sum((result.Nominal.StateTraj(i,data1:data2)-result.Tube.StateTraj(i,data1:data2)).^2)/length(result.Nominal.StateTraj(i,data1:data2)));
        Adapt_std(i+1,:) = sqrt(sum((result.Nominal.StateTraj(i,data1:data2)-result.Adapt.StateTraj(i,data1:data2)).^2)/length(result.Nominal.StateTraj(i,data1:data2)));
        Deep_std(i+1,:) = sqrt(sum((result.Nominal.StateTraj(i,data1:data2)-result.Deep.StateTraj(i,data1:data2)).^2)/length(result.Nominal.StateTraj(i,data1:data2)));
    end
end