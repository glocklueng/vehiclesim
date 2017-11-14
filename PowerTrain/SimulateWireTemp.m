clear wireModel log a b c
wireModel.simulation.parameter.deltaT = 0.1; % in seconds

wireModel.state.temperature.conductor = 20;   %C
wireModel.state.temperature.ambient = 20;     %C

load('Cloroplast_FHLR2GCB2G.mat')

wireModel = updateWireTemp(wireModel);

%time = (1:(3000 / wireModel.simulation.parameter.deltaT)) .* wireModel.simulation.parameter.deltaT;

load('driveCycle_Endurance_Single.mat');

simLength = length(time);
log(simLength)= wireModel;

for i = 1:simLength 
    wireModel.state.current = SingleMotorCurrentRequiredA(i);
    wireModel = updateWireTempFast(wireModel);
    log(i) = wireModel;
end

plot(time,arrayfun(@(x) x.state.temperature.conductor, log))
ylabel('Temperature (C)');
xlabel('Time (s)');
grid();