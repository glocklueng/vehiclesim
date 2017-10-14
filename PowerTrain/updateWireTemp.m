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

model = updateWireTempFast(model);
end

