

testCurrents = 0:10:400;
paramB = [20,85,125,140];
finalTemps = [];


for pb = 1:length(paramB)
for i = 1:length(testCurrents)
    clear wireModel log a b c
    wireModel.simulation.parameter.deltaT = 1; % in seconds
    
    wireModel.state.temperature.conductor = paramB(pb);   %C
    wireModel.state.temperature.ambient = paramB(pb);     %C
    
    wireModel.parameter.thicknessInsulation = 2.9e-3;
    wireModel.parameter.diamConductor = 8.5e-3;             %7.978846e-03;
    
    wireModel.parameter.kInsulation = 0.4;                   %W/(m?C)
    wireModel.parameter.hInsulationToAir =  10;               %W/(m^2?C) typically (2-25)
    wireModel.parameter.hWireToInsulation = 20;              %W/(m^2?C)
    
    wireModel.parameter.specificHeat = 385;                  %J/(Kg?C) copper
    wireModel.parameter.density = 8960*0.648;                %Kg/(m^3)
    
    wireModel.parameter.resistivity = 1.68e-8;
    
    wireModel = updateWireTemp(wireModel);
    
    time = (1:(3000 / wireModel.simulation.parameter.deltaT)) .* wireModel.simulation.parameter.deltaT;
    
    
    simLength = length(time);
    
    for j = 1:simLength
        wireModel.state.current = testCurrents(i);%SingleControllerInputCurrentA(i);
        wireModel = updateWireTempFast(wireModel);
        log(j) = wireModel;
    end
    finalTemps(i) = wireModel.state.temperature.conductor;
end
plot(testCurrents,finalTemps);
hold on;
end
% plot(time,arrayfun(@(x) x.state.temperature.conductor, log))
