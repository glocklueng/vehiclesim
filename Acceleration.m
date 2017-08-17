clear;

sim.dt = 0.001;
sim.len = 10;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

report.name_a = 'Car mass';
report.data_a = 250;%[200,220,240,260,280,300,320,340];
report.name_b = 'COF longitude';
report.data_b = 1.4;%[1.2,1.3,1.4,1.5,1.6,1.7,1.8];
report.name_c = 'Max torque';
report.data_c = 700;%[400,450,500,550,600,650,700,750,800];

report.table = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),5);
report.output_log = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),(sim.len/sim.dt));

for a = 1:length(report.data_a)
    for b = 1:length(report.data_b)
        for c = 1:length(report.data_c)
            
            car.m = report.data_a(a); %250
            car.cg = 0.3;
            car.wb = 1.6;
            car.swd = 0.5; %higher is more rearward
            car.u_long = report.data_b(b); %1.4
            car.u_lat = 1.5;
            car.r_tire = 0.26;
            car.n_mechanical = 0.95;
            car.n_electrical = 0.92;
            car.p_max = 80000*car.n_mechanical*car.n_electrical;
            car.t_max = report.data_c(c);
            car.a_f = 1.1;
            car.c_df = 1.87;
            car.c_d = 1.3;
            car.cp = 0.5;

            report.index = c+length(report.data_c)*(b-1)+length(report.data_c)*length(report.data_b)*(a-1);
            report.table(report.index,1) = report.data_a(a);
            report.table(report.index,2) = report.data_b(b);
            report.table(report.index,3) = report.data_c(c);
            
            %[acceleration, velocity, displacment]
            sim.state_i = [0;0;0];
            sim.state = zeros(length(sim.state_i),(sim.len/sim.dt));
            sim.state(:,1) = sim.state_i;

            for i = 2:length(sim.state);

                calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state(2,i-1)^2);
                calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state(2,i-1)^2);

                calc.wt = car.m*sim.state(1,i-1)*car.cg/car.wb;
                calc.f_frict_f = car.u_long*(param.g*car.m*(1-car.swd)+calc.f_down*(1-car.cp)-calc.wt);
                calc.f_frict_r = car.u_long*(param.g*car.m*(car.swd)+calc.f_down*(car.cp)+calc.wt);

                calc.f_motor = min(car.t_max/car.r_tire, car.p_max/sim.state(2,i-1));
                calc.f_cp_f = min(calc.f_frict_f);
                calc.f_cp_r = min(calc.f_frict_r,calc.f_motor);

                sim.state(1,i) = (calc.f_cp_r-calc.f_drag)/car.m;
                sim.state(2,i) = sim.state(2,i-1) + sim.state(1,i-1)*sim.dt;
                sim.state(3,i) = sim.state(3,i-1) + sim.state(2,i-1)*sim.dt;

                report.force_output(report.index,i) = calc.f_cp_r;
                report.power_output(report.index,i) = calc.f_cp_r*sim.state(2,i);
                report.traction_limit_r(report.index,i) = calc.f_frict_r;
                report.traction_limit_f(report.index,i) = calc.f_frict_f;
                report.motor_power(report.index,i) = calc.f_motor;
            end

            for i = 1:length(sim.state)
                if sim.state(2,i) >= 27.78
                    report.table(report.index,4) = i/1000;
                    break
                end
            end

            for i = 1:length(sim.state)
                if sim.state(3,i) >= 75
                    report.table(report.index,5) = i/1000;
                    break
                end
            end
            
        end
    end
end
figure(1);
plot(sim.state(2,:)*3.6,report.force_output(1,:));



