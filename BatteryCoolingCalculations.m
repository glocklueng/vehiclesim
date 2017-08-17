DCR = 10e-3;
heatCapacity =  1034.2; %kj/kg?k
mass = 0.496;

current = 200;
% tempRiseRate = ((DCR*current^2)/ (heatCapacity*mass))


tempRiseRate = @(current) ((DCR*current^2)/ (heatCapacity*mass));
timeToFailureAllowable = 6;

currentTemp = 35;
maxTemp = 50;

deltaT = maxTemp - currentTemp;

timeToFailure = deltaT/ tempRiseRate(current)
maxCurrent =  sqrt((((maxTemp - currentTemp)/timeToFailureAllowable) * (heatCapacity*mass))/DCR)

SOP =   min((maxCurrent) / 4,100)
