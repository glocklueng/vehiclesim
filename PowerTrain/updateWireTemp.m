function [model] = updateWireTemp(model)
model = verifyField(model,'simulation.parameter.deltaT',1e-3);
model = verifyField(model,'state.temperature.conductor',25);
model = verifyField(model,'state.temperature.ambient',25);
model = verifyField(model,'state.current',0);

model = verifyField(model,'parameter.diamConductor',9e-3);
model = verifyField(model,'parameter.thicknessInsulation',10e-03);
model = verifyField(model,'parameter.kInsulation',1);
model = verifyField(model,'parameter.hInsulationToAir',1);
model = verifyField(model,'parameter.hWireToInsulation',1);

model = verifyField(model,'parameter.specificHeat',385);
model = verifyField(model,'parameter.density',8960);
model = verifyField(model,'parameter.resistivity',1.68e-8);

% see http://www.mhtlab.uwaterloo.ca/courses/ece309/lectures/summary/summary_ch10&11_S16_309.pdf
rInsulation = log((model.parameter.thicknessInsulation+...
    (model.parameter.diamConductor/2))/(model.parameter.diamConductor/2))...
    /(2*pi*model.parameter.kInsulation);

rInsulationToAir = 1/(model.parameter.hInsulationToAir * (2*model.parameter.thicknessInsulation+model.parameter.diamConductor) * pi);
rWireToInsulation = 1/(model.parameter.hWireToInsulation * (model.parameter.diamConductor) * pi);
rTotal = rWireToInsulation + rInsulation + rInsulationToAir;

%assume 1 meter cable length

conductorArea = pi*(model.parameter.diamConductor/2)^2;
conductorResistance = model.parameter.resistivity/conductorArea;

heatGenerated = model.state.current^2 * conductorResistance;
heatDissipated = (model.state.temperature.conductor - model.state.temperature.ambient) / rTotal;

heatCapacity = (conductorArea * 1) * model.parameter.density*  model.parameter.specificHeat;

heatRate  = (heatGenerated - heatDissipated) / heatCapacity;
newTemp = model.state.temperature.conductor + heatRate*model.simulation.parameter.deltaT;
model.state.temperature.conductor = newTemp;
end

