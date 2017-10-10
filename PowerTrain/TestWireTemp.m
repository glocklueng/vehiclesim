wireModel.simulation.parameter.deltaT = 10; % in seconds


wireModel.state.temperature.conductor = 25;   %C
wireModel.state.temperature.ambient = 25;     %C

wireModel.parameter.thicknessInsulation = 850e-06;

wireModel.parameter.diamConductor = 8.5e-03;


wireModel.parameter.kInsulation = 0.4;                    %W/(m?C)
wireModel.parameter.hInsulationToAir = 11;               %W/(m?C) typically (2-25)
wireModel.parameter.hWireToInsulation = 18;              %W/(m^2?C)

wireModel.parameter.specificHeat = 385;           % J/(Kg?C) copper
wireModel.parameter.density = 8960;               %Kg/(m^3)
wireModel.parameter.resistivity = 1.68e-8;

wireModel.state.current = 293;
wireModel = updateWireTemp(wireModel)

for time = 1:500
    wireModel = updateWireTemp(wireModel);
    log(time) =wireModel;
end
wireModel.state.current = 0;
for time = 500:1000
    wireModel = updateWireTemp(wireModel);
    log(time) =wireModel;
end
a = [log.state]
b = [a.temperature]
c = [b.conductor];
plot(c)
