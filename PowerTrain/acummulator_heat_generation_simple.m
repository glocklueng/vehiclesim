%simple pack heat generation model

%% Heat generation in the cell
% assume no convective heat transfer from the surface of the cell 
% (valid assumption when cells are pressed up against each other)
clear all;
close all;
cell = BatteryCell(); %create instance of a battery cell
bb = Busbar();
%% Define Cell Parameters
        % specific heat taken from this paper: http://jes.ecsdl.org/content/162/1/A181.full
cell.specificHeatCapacity = 1243; % J kg^-1 k^-1
cell.ampacity = 6.3 * 60 * 60; %in amp seconds
cell.mass = 0.128; %kg
        
        
cell.R_ec_ref = 1.5e-3; %reference electrocemical ressistance @25C
cell.C_ec = 0.0005; %R_ec temp coefficent (need value for this)
cell.C_ocv = 0.003; %OCV temp coefficent (need value for this)

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

%% Set up test conditions
logCell(1) = cell;
logBB(1) = bb;

testCurrent = 100;%A
testTime = (logCell(1).ampacity/testCurrent); %s
testSize = 1000; %steps

timeStep = testTime/testSize;

thermalResistanceCellTab = 0.01; %w/c 

%% Run Test
for i = 2:testSize
cell = logCell(i-1);
bb = logBB(i-1);
cell.tabHeatFlux = (bb.temperature - cell.temperature)*thermalResistanceCellTab;
bb.tabHeatFlux = (cell.temperature - bb.temperature)*thermalResistanceCellTab;
logCell(i) = update(cell,testCurrent,timeStep);
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
plot((1:testSize)*timeStep,arrayfun(@(x)  x.temperature - 273.15,logCell))
hold on
plot((1:testSize)*timeStep,arrayfun(@(x)  x.temperature - 273.15,logBB))
title('Time vs Temperature');
xlabel('Time s');
ylabel('Temperature C');
legend({'battery' 'busbar'});
% figure(3)
% plot(arrayfun(@(x)  x.temperature - 273.15,logCell),arrayfun(@(x)  getImpedance(x),logCell))
% title('Temperature vs Impedance');
% xlabel('Temperature C');
% ylabel('Electrical Resistance R');