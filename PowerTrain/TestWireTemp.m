clear wireModel log a b c
wireModel.simulation.parameter.deltaT = 0.001; % in seconds

wireModel.state.temperature.conductor = 25;   %C
wireModel.state.temperature.ambient = 25;     %C

wireModel.parameter.thicknessInsulation = 850e-06;
wireModel.parameter.diamConductor = 8.5e-03;

wireModel.parameter.kInsulation = 0.4;                    %W/(m?C)
wireModel.parameter.hInsulationToAir = 11;               %W/(m^2?C) typically (2-25)
wireModel.parameter.hWireToInsulation = 18;              %W/(m^2?C)

wireModel.parameter.specificHeat = 385;           % J/(Kg?C) copper
wireModel.parameter.density = 8960;               %Kg/(m^3)
wireModel.parameter.resistivity = 1.68e-8;

wireModel = updateWireTemp(wireModel);
load('driveCycle_Endurance_Full.mat');

simLength = length(time);

for i = 1:simLength 
    wireModel.state.current = SingleControllerInputCurrentA(i);
    wireModel = updateWireTempFast(wireModel);
    log(i) = wireModel;
end

plot(time,arrayfun(@(x) x.state.temperature.conductor, log))
