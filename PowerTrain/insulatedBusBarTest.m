%simple pack heat generation model

%% Heat generation in the cell
% assume no convective heat transfer from the surface of the cell 
% (valid assumption when cells are pressed up against each other)
clear all;
close all;

bb = BusbarIns();

%% Define BusBar Parameters
bb.temperature = 298.15;%k
bb.width = 10e-3;%m
bb.height = 10e-3;%m
bb.length = 100e-3;%m
bb.density = 8960; %kg m^-3 @25C
bb.specificHeatCapacity = 385; % J kg^-1 k^-1
bb.resistivity = 1.68e-8;%?m
bb.resistivityAlpha = 0.00386;
bb.temperatureRef = 293.15;%k

bb.surfaceThermalConductivity = 5; 

%% Set up test conditions
logBB(1) = bb;

testCurrent = 200;%A
testTime = 800;%s
timeStep = 0.1;

testSize = testTime /timeStep ; %steps


thermalResistanceCellTab = 0.01; %w/c 

%% Run Test
for i = 2:testSize
bb = logBB(i-1);
logBB(i) = update(bb,4*testCurrent,timeStep);
end

%% Plot Results
% figure (1)
% plot([logCell.SOC],arrayfun(@(x)  terminalVoltage(x,40),logCell))
% title('SOC vs terminal Voltage');
% xlabel('SOC %');
% ylabel('terminal Voltag V')
% set ( gca, 'xdir', 'reverse' )
figure(2)
hold on
plot((1:testSize)*timeStep,arrayfun(@(x)  x.temperature - 273.15,logBB))
plot((1:testSize)*timeStep,arrayfun(@(x)  x.surfTemperature - 273.15,logBB))
title('Time vs Temperature');
xlabel('Time s');
ylabel('Temperature C');
legend({'copper' 'surface'});
% figure(3)
% plot(arrayfun(@(x)  x.temperature - 273.15,logCell),arrayfun(@(x)  getImpedance(x),logCell))
% title('Temperature vs Impedance');
% xlabel('Temperature C');
% ylabel('Electrical Resistance R');