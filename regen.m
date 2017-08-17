function [ torque, power ] = regen(speed,torqueLimit,motorStruct)
%REGEN Summary of this function goes here
%   input options:
%   regen(speed) 
%   regen(speed,torqueLimit)
%   regen(speed,torqueLimit,motorStruct)
%   speed in rads/s
%   torqueLimit in Nm
%   motorStruct:
%         motorData.torqueC = 0.363; % NM/A
%         motorData.emfC = 0.37626; % V*s/rad
%         motorData.maxCurrent = 460; % Amps
%         motorData.continuousCurrent = 125;
%         motorData.terminalResistance = 0.00972; %ohms



switch nargin
    case 1
        motorData.torqueC = 0.363; % NM/A
        motorData.emfC = 0.37626; % V*s/rad
        motorData.maxCurrent = 460; % Amps
        motorData.continuousCurrent = 125;
        motorData.terminalResistance = 0.00972; %ohms
        torqueLimit = inf
        
    case 2
        motorData.torqueC = 0.363; % NM/A
        motorData.emfC = 0.37626; % V*s/rad
        motorData.maxCurrent = 460; % Amps
        motorData.continuousCurrent = 125;
        motorData.terminalResistance = 0.00972; %ohms
        
    case 3
        motorData = motorStruct
end
torque.max = min(((speed.*motorData.emfC)./(motorData.terminalResistance *2)),motorData.maxCurrent).*motorData.torqueC;
torque.continuous = min(((speed.*motorData.emfC)./(motorData.terminalResistance *2)),motorData.continuousCurrent).*motorData.torqueC;
torque.max = min(torque.max,torqueLimit)
torque.continuous(torque.continuous,torqueLimit)

power.max = (min(torque.max,torqueLimit)/motorData.torqueC)* ((motorData.emfC*speed)- (min(torque.max,torqueLimit)/motorData.torqueC)*motorData.terminalResistance)
power.continuous = (min(torque.continuous,torqueLimit)/motorData.torqueC)* ((motorData.emfC*speed)- (min(torque.continuous,torqueLimit)/motorData.torqueC)*motorData.terminalResistance)
end

