close all
clear all
motor.torqueC = 0.363; % NM/A
motor.emfC = 0.37626; % V*s/rad
motor.maxCurrent = 460; % Amps
motor.terminalResistance = 0.00972; %ohms

maxBreakTorque = @(speed) ((speed*motor.emfC)/motor.terminalResistance) * motor.torqueC;
speed = 500; %rad/s
BEMF = speed*motor.emfC



regenPower = @(loadImpedance) ((speed.*motor.emfC)./(motor.terminalResistance + loadImpedance).^2 .*loadImpedance);
regenTorqe = @(loadImpedance) ((speed.*motor.emfC)./(motor.terminalResistance + loadImpedance).*motor.torqueC);

plot([0.000001:0.00001:0.03],regenPower([0.000001:0.00001:0.03]))
hold on
plot([0.000001:0.00001:0.03],regenTorqe([0.000001:0.00001:0.03]))

