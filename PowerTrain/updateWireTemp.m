function [model] = updateWireTemp(model)
deltaT = 1e-3; % in seconds

thicknessInsulation = 10e-03;       %meters
conductorDiam = 9e-3;               %meters

kInsulation = 1;                    %W/(m?C)
hInsulationToAir = 1;               %W/(m?C)
hWireToInsulation = 1;              %W/(m^2?C)

specificHeatCopper = 385;           % J/(Kg?C) copper
densityCopper = 8960;               %Kg/(m^3)

% see http://www.mhtlab.uwaterloo.ca/courses/ece309/lectures/summary/summary_ch10&11_S16_309.pdf
rInsulation = log((thicknessInsulation+(conductorDiam/2))/(conductorDiam/2))...
    /(2*pi*kInsulation);

rInsulationToAir = 1/(hInsulationToAir * (2*thicknessInsulation+conductorDiam) * pi);
rWireToInsulation = 1/(hWireToInsulation * (conductorDiam) * pi);
rTotal = rWireToInsulation + rInsulation + rInsulationToAir;

%assume 1 meter cable length
resistivityM = 1.68e-8;
conductorArea = pi*(conductorDiam/2)^2;
conductorResistance = resistivityM/conductorArea;

heatGenerated = model.current^2 * conductorResistance;
heatDissipated = (model.temperature.conductor - model.temperature.ambient) / rTotal;

heatCapacity = (conductorArea * 1) * densityCopper *  specificHeatCopper;

heatRate  = (heatGenerated - heatDissipated) / heatCapacity;
newTemp = model.temperature.conductor + heatRate*deltaT;
model.temperature.conductor = newTemp;
end

