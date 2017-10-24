classdef BatteryCell
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
   
        SOC = 1;
        temperature = 298.15;
        
        tabHeatFlux = 0; %heat in watts entering the battery through the terminals
        
        % specific heat taken from this paper: http://jes.ecsdl.org/content/162/1/A181.full
        specificHeatCapacity = 1243; % J kg^-1 k^-1
        ampacity = 6.3 * 60 * 60; %in amp seconds
        mass = 0.128; %kg
        
        
        R_ec_ref = 1.5e-3; %reference electrocemical ressistance
        C_ec = 0.0005; %R_ec temp coefficent (need value for this)
        
        temperature_ref = 298.15;
        
        OCV_max = 4.2;
        OCV_min = 3.0;
        C_ocv = 0.003; %OCV temp coefficent (need value for this)
        
        
    end
    
    methods
        
        function  [imp] = getImpedance(cell)
            imp =  cell.R_ec_ref * ((cell.temperature/cell.temperature_ref)*...
                exp((1/cell.C_ec)*((1/cell.temperature) - (1/cell.temperature_ref)))...
                + 0.5*(1-cell.SOC));
            
            
        end
        
        function cell = update(cell,current,timeStep)
            heatGenerated = getImpedance(cell) * current^2 * timeStep;
            deltaTemp =  (cell.tabHeatFlux+heatGenerated) / (cell.specificHeatCapacity * cell.mass);
            
            cell.SOC = cell.SOC - current*timeStep / cell.ampacity;
            cell.temperature =  deltaTemp + cell.temperature;
        end
        

        function  [OCV] = getOCV(cell)
            OCV_ref = cell.OCV_min + cell.SOC*(cell.OCV_max - cell.OCV_min); %this is super crude
            OCV = OCV_ref *(1 + cell.C_ocv*(cell.temperature - cell.temperature_ref));
        end
        
       function  [V_term] = terminalVoltage(cell,current)
            V_term = getOCV(cell) - current * getImpedance(cell);
        end
       
        
    end
    
end

