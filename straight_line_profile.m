clear;
syms r v;

sim.dt = 0.001;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

nonideal.corner_traction = 1;
nonideal.uniform_braking = 0.95;

report.name_a = 'Car mass';
report.data_a = 260;%[240,260,280,300];
report.name_b = 'COF longitude';
report.data_b = 1.7;%[1.2,1.3,1.4,1.5,1.6,1.7,1.8];
report.name_c = 'Max torque';
report.data_c = 1000;%[500,600,700,800,900,1000];
report.index = 0;
report.table = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),5);

number_of_laps = 2;
log.activate = 1;
log.number_of_laps = 2;
log.lap_data = zeros(4,400000); %very large
log.lap_data_index = 1;


% track.input = [
% 20 ,10; 
% 30 ,0; 
% 10 ,20; 
% 40 ,0; 
% 8  ,8; 
% 6  ,120; 
% 50 ,0];

track.input = [
037.700, 000.000;
017.900, 030.200;
013.100, 012.400;
008.900, 009.900;
012.100, 015.100;
010.000, 020.000;
012.200, 021.100;
016.000, 188.000;
050.800, 031.900;
029.100, 000.000;
013.300, 040.300;
013.600, 017.700;



track.data = zeros(length(track.input(:,1)),3);
track.data(:,1:2) = track.input;

for a = 1:length(report.data_a)
    for b = 1:length(report.data_b)
        for c = 1:length(report.data_c)
            
            car.m = report.data_a(a); %250
            car.cg = 0.3;
            car.wb = 1.6;
            car.swd = 0.5;%0.6; %higher is more rearward
            car.cp = 0.5;
            car.bb = 0.3; %higher is more rear braking
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
            
            
            report.index = report.index + 1;
            report.table(report.index,1) = report.data_a(a);
            report.table(report.index,2) = report.data_b(b);
            report.table(report.index,3) = report.data_c(c);
            
            eqn = ((0.5*car.m*v^2/r)/(0.5*car.u_lat*(car.m*param.g+0.5*param.air_p*car.a_f*car.c_df*v^2)))^2 + ((0.5*param.air_p*car.a_f*car.c_d*v^2)/(0.5*car.u_long*(car.m*param.g+0.5*param.air_p*car.a_f*car.c_df*v^2)))^2 == nonideal.corner_traction;
            for i = 1:length(track.data(:,1))
                if track.data(i,2) ~= 0
                    track.data(i,3) = max(real(double(solve(subs(eqn,r,track.data(i,2)),v))));
                else
                    track.data(i,3) = 0;
                end
            end

            %initialize to zero speed
            sim.state_p = [0;0;0];
            sim.state_c = sim.state_p;
            sim.state_index_f = 1;
            sim.state_index_r = 1;
            sim.state_block_f = zeros(3,15/sim.dt);
            sim.state_block_r = zeros(3,15/sim.dt);
            
            %run race
            for lap_i = 1:number_of_laps
                for track_i = 1:length(track.data(:,1))
                    sim.state_index_f = 1;
                    sim.state_index_r = 1;
                    %run forwards accross the whole track segment, will
                    %figure out braking if we are too fast going into the
                    %next segment
                    while sim.state_p(3) < track.data(track_i,1)
                        if track.data(track_i,2) ~= 0 %Calculate things for this timestep
                            calc.f_lat = car.m*sim.state_p(2)^2/track.data(track_i,2);
                        else
                            calc.f_lat = 0;
                        end
                        %calc.wt = car.m*sim.state_p(1)*car.cg/car.wb;%Weight transfer, used for friction calc
                        calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2)^2);
                        calc.f_n = param.g*car.m+calc.f_down;
                        %calc.f_n_f = param.g*car.m*(1-car.swd)+calc.f_down*(1-car.cp)-calc.wt;
                        %calc.f_n_r = param.g*car.m*(car.swd)+calc.f_down*(car.cp)+calc.wt;
                        calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                        %calc.f_max_long_f = sqrt( ((car.u_long*calc.f_n_f)^2) - ((((calc.f_lat/2)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                        %calc.f_max_long_r = sqrt( ((car.u_long*calc.f_n_r)^2) - ((((calc.f_lat/2)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                        calc.f_max_motor = min(car.t_max/car.r_tire, car.p_max/sim.state_p(2));
                        calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state_p(2)^2);
                        calc.f_cp_r = min(calc.f_max_long/2,calc.f_max_motor);

                        sim.state_c(1) = (calc.f_cp_r-calc.f_drag)/car.m;
                        sim.state_c(2) = sim.state_p(2) + sim.state_c(1)*sim.dt;
                        sim.state_c(3) = sim.state_p(3) + sim.state_c(2)*sim.dt;
                        
                        sim.state_block_f(:,sim.state_index_f) = sim.state_c;

                        sim.state_index_f = sim.state_index_f + 1;
                        sim.state_p = sim.state_c;
                    end
                    sim.state_index_f = sim.state_index_f - 1;
                    %check to see if we are too fast for the new segment
                    if and(track_i >= length(track.data(:,1)),lap_i >= number_of_laps)
                        %add to report, finished
                        if and(log.activate == 1, lap_i <= log.number_of_laps)
                            log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(:,1:sim.state_index_f);
                            log.lap_data_index = log.lap_data_index+sim.state_index_f;
                        end
                        %we are done here
                        break;
                    elseif and(track.data(mod(track_i,length(track.data(:,1)))+1,3) ~= 0, sim.state_c(2) >= track.data(mod(track_i,length(track.data(:,1)))+1,3))
                        %we need to brake before entering the next track section
                        sim.state_c = [0;track.data(mod(track_i,length(track.data(:,1)))+1,3);track.data(track_i,1)];
                        sim.state_p = sim.state_c;

                        while sim.state_p(3) > 0
                            if track.data(track_i,2) ~= 0 %Calculate things for this timestep
                                calc.f_lat = car.m*sim.state_c(2)^2/track.data(track_i,2);
                            else
                                calc.f_lat = 0;
                            end
                            %calc.wt = car.m*sim.state_c(1)*car.cg/car.wb;%Weight transfer, used for friction calc
                            calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_c(2)^2);
                            calc.f_n = param.g*car.m+calc.f_down;
                            %calc.f_n_f = param.g*car.m*(1-car.swd)+calc.f_down*(1-car.cp)-calc.wt;
                            %calc.f_n_r = param.g*car.m*(car.swd)+calc.f_down*(car.cp)+calc.wt;
                            calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                            %calc.f_max_long_f = sqrt((car.u_long*(calc.f_n_f)^2)-(((calc.f_lat^2)*(car.u_long*(calc.f_n_f)^2))/((car.u_lat*(calc.f_n_f))^2)));
                            %calc.f_max_long_r = sqrt((car.u_long*(calc.f_n_r)^2)-(((calc.f_lat^2)*(car.u_long*(calc.f_n_r)^2))/((car.u_lat*(calc.f_n_r))^2)));
                            calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state_c(2)^2);
                            calc.f_cp = calc.f_max_long*nonideal.uniform_braking;
                            %calc.f_cp_f = min(calc.f_max_long_f,calc.f_max_long_r*car.bb/(1-car.bb));
                            %calc.f_cp_r = min(calc.f_max_long_r,calc.f_max_long_f*(1-car.bb)/car.bb);

                            sim.state_p(1) = (-calc.f_cp-calc.f_drag)/car.m;
                            sim.state_p(2) = sim.state_c(2) - sim.state_p(1)*sim.dt;
                            sim.state_p(3) = sim.state_c(3) - sim.state_p(2)*sim.dt;

                            sim.state_block_r(:,sim.state_index_r) = sim.state_c;

                            sim.state_index_r = sim.state_index_r + 1;
                            sim.state_c = sim.state_p;


                            while sim.state_p(3) < sim.state_block_f(3,sim.state_index_f)
                                sim.state_index_f = sim.state_index_f-1;
                            end
                            if sim.state_p(2) > sim.state_block_f(2,sim.state_index_f)
                                %found a match
                                break;
                            end
                        end
                        %fix counter
                        sim.state_index_r = sim.state_index_r - 1;
                        %initialize next inital velocity
                        sim.state_p(1:3) = [0,sim.state_block_r(2,1),0];
                        sim.state_c = sim.state_p;
                        %add to report, with braking
                        if and(log.activate == 1, lap_i <= log.number_of_laps)
                            log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(:,1:sim.state_index_f);
                            log.lap_data_index = log.lap_data_index+sim.state_index_f;
                            log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_r-1)) = fliplr(sim.state_block_r(:,1:sim.state_index_r));
                            log.lap_data_index = log.lap_data_index+sim.state_index_r;
                        end
                    else %no braking was required before next segment
                        %initialize next inital velocity
                        sim.state_p(1:3) = [0,sim.state_block_f(2,sim.state_index_f),0];
                        sim.state_c = sim.state_p;
                        %add to report, without braking section
                        if and(log.activate == 1, lap_i <= log.number_of_laps)
                            log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(:,1:sim.state_index_f);
                            log.lap_data_index = log.lap_data_index+sim.state_index_f;
                        end
                    end
                end
                
                plot(log.lap_data(2,1:log.lap_data_index));
                
            end
        end
    end
end
% hold off;
% figure(1);
% plot(sim.state(2,:)*3.6,report.traction_limit_f(:),sim.state(2,:)*3.6,report.traction_limit_r(:));
% plot(sim.state(2,:)*3.6,report.force_output(:), sim.state(2,:)*3.6,report.traction_limit(:),sim.state(2,:)*3.6,report.motor_power(:));
% figure(2);
% plot(sim.state(2,:)*3.6,report.power_output(:));

