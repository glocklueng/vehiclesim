clear;

sim.dt = 0.001;
sim.len = 100;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

report.name_a = 'Car mass';
report.data_a = 250;%[200,220,240,260,280,300,320,340];
report.name_b = 'COF longitude';
report.data_b = 1.4;%[1.2,1.3,1.4,1.5,1.6,1.7,1.8];
report.name_c = 'Max torque';
report.data_c = 3;%[400,450,500,550,600,650,700,750,800];

report.table = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),5);
report.output_log = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),(sim.len/sim.dt));

for a = 1:length(report.data_a)
    for b = 1:length(report.data_b)
        for c = 1:length(report.data_c)
            
            car.m = report.data_a(a); %250
            car.cg = 0.3;
            car.wb = 1.6;
            car.swd = 0.5; %higher is more rearward
            car.a_f = 1.1;
            car.c_df = 1.87;
            car.c_d = 1.3;
            car.cp = 0.5;
            car.u_long = report.data_b(b); %1.4
            car.u_lat = 1.5;
            car.r_tire = 0.265;
            %Motor and gearbox
            car.n_mech = 0.95;
            car.n_elec = 0.92;
            car.p_max = 80000;
            %Emrax 188 MV
            car.nm = 2;
            car.t_max = 90;
            car.n_max = 6000;
            car.gr = report.data_c(c);
            car.v_max = (car.n_max/car.gr/60*2*pi)*car.r_tire;

            report.index = c+length(report.data_c)*(b-1)+length(report.data_c)*length(report.data_b)*(a-1);
            report.table(report.index,1) = report.data_a(a);
            report.table(report.index,2) = report.data_b(b);
            report.table(report.index,3) = report.data_c(c);
            
            %[acceleration, velocity, displacment]
            %point mass
            sim.state_p_i = [0;0;0];
            sim.state_p = zeros(length(sim.state_p_i),(sim.len/sim.dt));
            sim.state_p(:,1) = sim.state_p_i;
            %bicycle model
            sim.state_b_i = [0;0;0];
            sim.state_b = zeros(length(sim.state_b_i),(sim.len/sim.dt));
            sim.state_b(:,1) = sim.state_b_i;

            for i = 2:(sim.len/sim.dt);

                calc.f_down_b  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_b(2,i-1)^2);
                calc.f_drag_b  = (0.5*param.air_p*car.a_f*car.c_d *sim.state_b(2,i-1)^2);
                calc.f_down_p  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2,i-1)^2);
                calc.f_drag_p  = (0.5*param.air_p*car.a_f*car.c_d *sim.state_p(2,i-1)^2);

                calc.wt = car.m*sim.state_b(1,i-1)*car.cg/car.wb;
                calc.f_frict_f = car.u_long*(param.g*car.m*(1-car.swd)+calc.f_down_b*(1-car.cp)-calc.wt);
                calc.f_frict_r = car.u_long*(param.g*car.m*(car.swd)+calc.f_down_b*(car.cp)+calc.wt);
                calc.f_frict   = car.u_long*(param.g*car.m+calc.f_down_p);
                

                calc.f_motor_p = min(car.nm*car.t_max*car.gr/car.r_tire,...
                                    (car.p_max*car.n_mech*car.n_elec/sim.state_p(2,i-1))*(1.5^(1/(sim.state_p(2,i-1)-car.v_max))));
                calc.f_motor_b = min(car.nm*car.t_max*car.gr/car.r_tire,...
                                    (car.p_max*car.n_mech*car.n_elec/sim.state_b(2,i-1))*(1.5^(1/(sim.state_b(2,i-1)-car.v_max))));
                
                calc.f_cp_f = min(calc.f_frict_f);
                calc.f_cp_r = min(calc.f_frict_r,calc.f_motor_b);
                calc.f_cp   = min(calc.f_frict/2,calc.f_motor_p);

                sim.state_b(1,i) = (calc.f_cp_r-calc.f_drag_b)/car.m;
                sim.state_b(2,i) = sim.state_b(2,i-1) + sim.state_b(1,i-1)*sim.dt;
                sim.state_b(3,i) = sim.state_b(3,i-1) + sim.state_b(2,i-1)*sim.dt;
                
                sim.state_p(1,i) = (calc.f_cp-calc.f_drag_p)/car.m;
                sim.state_p(2,i) = sim.state_p(2,i-1) + sim.state_p(1,i-1)*sim.dt;
                sim.state_p(3,i) = sim.state_p(3,i-1) + sim.state_p(2,i-1)*sim.dt;

                report.force_output_b(report.index,i) = calc.f_cp_r;
                report.force_output_p(report.index,i) = calc.f_cp;
                
                report.traction_limit_b(report.index,i) = calc.f_frict_r;
                report.traction_limit_p(report.index,i) = calc.f_frict/2;
                
                report.motor_power_b(report.index,i) = calc.f_motor_b;
                report.motor_power_p(report.index,i) = calc.f_motor_p;
            end

            for i = 1:(sim.len/sim.dt)
                if sim.state_p(3,i) >= 75
                    report.table(report.index,4) = i/1000;
                    break
                end
            end
            for i = 1:(sim.len/sim.dt)
                if sim.state_b(3,i) >= 75
                    report.table(report.index,5) = i/1000;
                    break
                end
            end
            
        end
    end
end
figure('name','Bicycle Model');
plot(sim.state_b(2,:)*3.6,report.force_output_b(1,:), sim.state_b(2,:)*3.6,report.traction_limit_b(1,:), sim.state_b(2,:)*3.6,report.motor_power_b(1,:));
figure('name','Point Mass Model');
plot(sim.state_p(2,:)*3.6,report.force_output_p(1,:), sim.state_p(2,:)*3.6,report.traction_limit_p(1,:), sim.state_p(2,:)*3.6,report.motor_power_p(1,:));



