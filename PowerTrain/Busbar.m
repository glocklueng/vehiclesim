classdef Busbar
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        temperature = 298.15;%K
        current = 0; %A
        width = 7e-3; %m
        height = 7e-3;%m
        length = 100e-3;%m
        density = 8960; %kg m^-3 @25C
        mass = 0; %kg
        tabHeatFlux = 0;%w
        specificHeatCapacity = 385; % J kg^-1 k^-1
        resistivity = 1.68e-8;%?m
        resistivityAlpha = 0.00386;
        temperatureRef = 293.15;%k
    end
    
    methods
        
 function  [imp] = getImpedance(bb)
            imp =  (bb.length*getResistivity(bb))/(bb.width*bb.height);
        end
        
        function bb = update(bb,current,timeStep)
            heatGenerated = getImpedance(bb) * current^2 * timeStep;
            bb.mass =  bb.width*bb.length*bb.height * bb.density;
            deltaTemp =  (bb.tabHeatFlux+heatGenerated) / (bb.specificHeatCapacity * bb.mass);
            bb.temperature =  deltaTemp + bb.temperature;
        end
        
        function res = getResistivity(bb)
            res = bb.resistivity*(1 + bb.resistivityAlpha*(bb.temperature - bb.temperatureRef)); 
        end
        
    end
    
end

