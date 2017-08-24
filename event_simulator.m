clear;
syms r v;

sim.dt = 0.001;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

nonideal.corner_traction = 1;
nonideal.uniform_braking = 1;

track.input = [
 0, 037.700, 000.000;
 1, 017.900, 030.200;
 1, 013.100, 012.400;
-1, 008.900, 009.900;
 1, 012.100, 015.100;
-1, 010.000, 020.000;
 1, 012.200, 021.100;
-1, 016.000, 188.000;
 1, 050.800, 031.900;
 0, 029.100, 000.000;
 1, 013.300, 040.300;
-1, 013.600, 017.700;
 1, 016.000, 017.200;
-1, 017.600, 019.100;
 1, 015.600, 050.800;
 0, 047.400, 000.000;
 1, 021.300, 021.400;
-1, 029.600, 015.800;
 0, 002.500, 000.000;
 1, 021.400, 008.300;
 0, 060.000, 000.000;
 1, 015.800, 024.800;
-1, 016.000, 013.700;
 1, 011.100, 012.400;
-1, 012.300, 015.900;
 1, 016.100, 022.900;
-1, 008.100, 015.000;
 0, 011.000, 000.000;
 1, 031.900, 013.900;
-1, 035.200, 039.100;
 0, 013.100, 000.000;
 1, 051.100, 025.600;
 1, 023.000, 052.000;
 1, 032.300, 019.500;
 0, 010.100, 000.000;
-1, 012.800, 010.000;
-1, 029.700, 028.700;
 0, 030.700, 000.000;
 1, 012.300, 017.300;
-1, 018.400, 008.800;
 0, 061.600, 000.000;
 1, 019.900, 020.500;
-1, 014.200, 008.100;
 1, 029.700, 022.500;
 0, 017.600, 000.000;
-1, 047.200, 017.200;
-1, 025.300, 020.900;
 0, 006.300, 000.000;
 1, 016.000, 010.900;
 0, 005.900, 000.000;
-1, 012.800, 009.700;
 1, 020.000, 010.000;
 1, 022.600, 018.600;
 0, 013.600, 000.000;];

track.number_of_laps = 19;
track.data = zeros(length(track.input(:,1)),4);
track.data(:,1:2) = track.input(:,2:3);
track.traversed = 0;

log.plot = 1;
log.lap_data = zeros(5,200000);
log.accel_plot = zeros(3,20/sim.dt);
log.autocross_times = zeros(length(track.input(:,1)),1);
log.endurance_times = zeros(length(track.input(:,1)),2);
log.endurance_power = zeros(length(track.input(:,1)),2);

%report generates a table of permutation results for all parameters as well
%as the time to complete endurance and the energy comsumption durring that
%endurance run.
report.data_a = 320;%[200,220,240,260,280];
report.data_b = 1.6;%[1.7,1.1,1.2,1.3,1.4,1.5,1.6,1.7];
report.data_c = 5.5;%[2,2.25,2.5,2.75,3,3.25,3.5,3.75,4,4.25,4.5,4.75,5,5.25,5.5,5.75,6,6.25,6.5];
report.data_d = [30000,35000,40000,45000,50000,55000,60000,65000,70000,75000,80000];
report.result_index = 4; %amount of vector data
report.table = zeros(length(report.data_a)*length(report.data_b)*length(report.data_c),report.result_index+4);
report.index = 0;

for a = 1:length(report.data_a)
    for b = 1:length(report.data_b)
        for c = 1:length(report.data_c)
            for d = 1:length(report.data_d)
                %Set and record parameters
                report.index = report.index + 1;
                report.table(report.index,1) = report.data_a(a);
                report.table(report.index,2) = report.data_b(b);
                report.table(report.index,3) = report.data_c(c);
                report.table(report.index,4) = report.data_d(d);

                car.m = report.data_a(a);
                car.cg = 0.3;
                car.wb = 1.6;
                car.swd = 0.6; %higher is more rearward
                car.cp = 0.5;
                %car.bb = 0.3; %higher is more rear braking
                car.u_long = report.data_b(b);
                car.u_lat = 1.5;
                car.r_tire = 0.265;
                %Aero
                car.a_f = 1.24;
                car.c_df = 1.61;
                car.c_d = 0.96;
                %Motor and gearbox
                car.n_mech = 0.95;
                car.n_elec = 0.92;
                car.p_max = 80000;
                car.p_max_endurance = report.data_d(d);

                %Emrax 188 MV
                car.nm = 2;
                car.t_max = 90; %actually 100
                car.n_max = 7000;
    %             %Emrax 208 MV
    %             car.nm = 2;
    %             car.t_max = 135; %actually 150
    %             car.n_max = 6000;
    %             %Emrax 228 MV
    %             car.nm = 1;
    %             car.t_max = 220; %actually 240
    %             car.n_max = 5500;

                car.gr = report.data_c(c);
                car.v_max = (car.n_max/car.gr/60*2*pi)*car.r_tire;

                %==============================================================
                %Acceleration Event
                %==============================================================
                sim.state_p = [0;0;0];%initialize to zero speed
                sim.state_index_f = 1;
                sim.state_c = sim.state_p;

                while sim.state_c(3) < 75

                    calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2)^2);
                    calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d *sim.state_p(2)^2);
                    calc.wt = car.m*sim.state_p(1)*car.cg/car.wb;
                    calc.f_frict_f = car.u_long*(param.g*car.m*(1-car.swd)+calc.f_down*(1-car.cp)-calc.wt);
                    calc.f_frict_r = car.u_long*(param.g*car.m*(car.swd)+calc.f_down*(car.cp)+calc.wt);
                    calc.f_max_motor = min(car.nm*car.t_max*car.gr/car.r_tire,...
                    (car.p_max*car.n_mech*car.n_elec/sim.state_p(2))*(1.5^(1/(sim.state_p(2)-car.v_max))));
                    calc.f_cp_r = min(calc.f_frict_r,calc.f_max_motor);

                    sim.state_c(1) = (calc.f_cp_r-calc.f_drag)/car.m;
                    sim.state_c(2) = sim.state_p(2) + sim.state_c(1)*sim.dt;
                    sim.state_c(3) = sim.state_p(3) + sim.state_c(2)*sim.dt;
                    if log.plot == 1
                        log.accel_data(:,sim.state_index_f) = sim.state_c;
                        sim.state_index_f = sim.state_index_f + 1;
                    end
                    sim.state_p = sim.state_c;
                end
                report.table(report.index,report.result_index+1) = sim.state_index_f/1000;


                %==============================================================
                %Autocross and Endurance, calculate track
                %==============================================================
                %Need to back calculate the max input segment speed for the
                %whole track. Start by looking at the max speed according to
                %radius of curvature, then make sure that those speed can be
                %reached by reverse brakinging from each segment backwards
                %around the track until the steady state solution is found
                eqn = ((0.5*car.m*v^2/r)/(0.5*car.u_lat*(car.m*param.g+0.5*param.air_p*car.a_f*car.c_df*v^2)))^2 + ((0.5*param.air_p*car.a_f*car.c_d*v^2)/(0.5*car.u_long*(car.m*param.g+0.5*param.air_p*car.a_f*car.c_df*v^2)))^2 == nonideal.corner_traction;
                eqn = solve(eqn,v);
                for i = 1:length(track.data(:,1))
                    if track.data(i,2) ~= 0
                        track.data(i,3) = max(real(double(subs(eqn,r,track.data(i,2)))));
                        track.data(i,4) = track.data(i,3);
                    else
                        track.data(i,3) = 0;
                        track.data(i,4) = 0;
                    end
                end
                track.track_i = length(track.data(:,1));
                while true
                    track.v_max_c = track.data(mod(track.track_i-1,length(track.data(:,1)))+1,3);
                    track.v_max_n = track.data(mod(track.track_i,length(track.data(:,1)))+1,3);
                    %if this segment is not behind a non restricted section and
                    %it is straight or has a required speed greater than the
                    %future speed, then we need to update
                    if and(track.v_max_n ~= 0,or(track.v_max_c == 0,and(track.v_max_c ~= 0, track.v_max_c > track.v_max_n)));
                        sim.state_c = [0;track.v_max_n;track.data(mod(track.track_i-1,length(track.data(:,1)))+1,1)];
                        sim.state_p = sim.state_c;
                        while sim.state_p(3) > 0
                            sim.state_c = sim.state_p;
                            %calculate things
                            if track.data(mod(track.track_i-1,length(track.data(:,1)))+1,2) ~= 0 %Calculate things for this timestep
                                calc.f_lat = car.m*sim.state_c(2)^2/track.data(mod(track.track_i-1,length(track.data(:,1)))+1,2)*nonideal.corner_traction;
                            else
                                calc.f_lat = 0;
                            end
                            calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_c(2)^2);
                            calc.f_n = param.g*car.m+calc.f_down;
                            calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                            calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state_c(2)^2);
                            calc.f_cp = calc.f_max_long*nonideal.uniform_braking;
                            sim.state_p(1) = (-calc.f_cp-calc.f_drag)/car.m;
                            sim.state_p(2) = sim.state_c(2) - sim.state_c(1)*sim.dt;
                            sim.state_p(3) = sim.state_c(3) - sim.state_c(2)*sim.dt;
                        end
                        %update our required speed for the entrence of this segment
                        %to ensure that we can brake for the next one.   
                        if track.v_max_c == 0 %make it the new number
                            track.data(mod(track.track_i-1,length(track.data(:,1)))+1,3) = 0.99*sim.state_c(2);
                        else %take the min
                            track.data(mod(track.track_i-1,length(track.data(:,1)))+1,3) = min(track.v_max_c,0.99*sim.state_c(2));
                        end
                    elseif track.track_i <= 0
                        break;
                    end
                    track.track_i = track.track_i - 1;
                end

                %==============================================================
                %Autocross
                %==============================================================
                sim.state_p = [0;0;0];%initialize to zero speed
                sim.state_block_f = zeros(3,20/sim.dt);
                sim.state_block_r = zeros(3,20/sim.dt);
                sim.state_c = sim.state_p;
                sim.state_index_f = 1;
                sim.state_index_r = 1;
                log.lap_data_index = 1;

                for track_i = 1:length(track.data(:,1))
                    track.track_i = track_i;
                    sim.state_index_f = 1;
                    sim.state_index_r = 1;
                    %run forwards accross the whole track segment, will
                    %figure out braking if we are too fast going into the
                    %next segment
                    while sim.state_p(3) < track.data(track.track_i,1)
                        if track.data(track.track_i,2) ~= 0 %Calculate things for this timestep
                            calc.f_lat = car.m*sim.state_p(2)^2/track.data(track.track_i,2);
                        else
                            calc.f_lat = 0;
                        end
                        calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2)^2);
                        calc.f_n = param.g*car.m+calc.f_down;
                        calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                        calc.f_max_motor = min(car.nm*car.t_max*car.gr/car.r_tire,...
                        (car.p_max*car.n_mech*car.n_elec/sim.state_p(2))*(1.5^(1/(sim.state_p(2)-car.v_max))));
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
                    %this is called if we need to perform braking to obide by
                    %the maximum section transfer speed.
                    if and(track.data(mod(track.track_i,length(track.data(:,1)))+1,3) ~= 0, sim.state_c(2) >= track.data(mod(track.track_i,length(track.data(:,1)))+1,3))
                        %we need to brake before entering the next track section
                        sim.state_c = [0;track.data(mod(track.track_i,length(track.data(:,1)))+1,3);track.data(track.track_i,1)];
                        sim.state_p = sim.state_c;

                        while sim.state_p(3) > 0
                            if track.data(track.track_i,2) ~= 0 %Calculate things for this timestep
                                calc.f_lat = car.m*sim.state_c(2)^2/track.data(track.track_i,2);
                            else
                                calc.f_lat = 0;
                            end
                            calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_c(2)^2);
                            calc.f_n = param.g*car.m+calc.f_down;
                            calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );
                            calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state_c(2)^2);
                            calc.f_cp = calc.f_max_long*nonideal.uniform_braking;
                            sim.state_p(1) = (-calc.f_cp-calc.f_drag)/car.m;
                            sim.state_p(2) = sim.state_c(2) - sim.state_c(1)*sim.dt;
                            sim.state_p(3) = sim.state_c(3) - sim.state_c(2)*sim.dt;
                            sim.state_block_r(:,sim.state_index_r) = sim.state_c;
                            sim.state_index_r = sim.state_index_r + 1;
                            sim.state_c = sim.state_p;

                            while sim.state_p(3) <= sim.state_block_f(3,sim.state_index_f)
                                sim.state_index_f = sim.state_index_f-1;
                            end
                            if sim.state_p(2) >= sim.state_block_f(2,sim.state_index_f)
                                %found a match
                                break;
                            end
                        end
                        %fix counter
                        sim.state_index_r = sim.state_index_r - 1;
                        %initialize next inital velocity
                        sim.state_p = [0;sim.state_block_r(2,1);0];
                        sim.state_c = sim.state_p;
                        %Update all logging in the braking case
                        log.autocross_times(track.track_i) = (sim.state_index_f+sim.state_index_r)/1000;
                    else %no braking was required before next segment
                        %initialize next inital velocity
                        sim.state_p = [0;sim.state_block_f(2,sim.state_index_f);0];
                        sim.state_c = sim.state_p;
                        %Update all logging in the non braking case
                        log.autocross_times(track.track_i) = sim.state_index_f/1000;
                    end
                    track.traversed = track.traversed + track.data(track.track_i,1);
                end
                %calculate metrics and fill the report in
                report.table(report.index,report.result_index+2) = sum(log.autocross_times(:));


                %==============================================================
                %Endurance
                %==============================================================
                sim.state_p = [0;0;0];%initialize to zero speed
                sim.state_block_f = zeros(3,20/sim.dt);
                sim.state_block_r = zeros(3,20/sim.dt);
                sim.power_block_f = zeros(1,20/sim.dt);
                %sim.power_block_r = zeros(1,20/sim.dt);
                for lap_type = 1:2
                    sim.state_c = sim.state_p;
                    sim.state_index_f = 1;
                    sim.state_index_r = 1;
                    log.lap_data_index = 1;
                    for track_i = 1:length(track.data(:,1))
                        track.track_i = track_i;
                        sim.state_index_f = 1;
                        sim.state_index_r = 1;
                        %run forwards accross the whole track segment, will
                        %figure out braking if we are too fast going into the
                        %next segment
                        while sim.state_p(3) < track.data(track.track_i,1)
                            if track.data(track.track_i,2) ~= 0 %Calculate things for this timestep
                                calc.f_lat = car.m*sim.state_p(2)^2/track.data(track.track_i,2);
                            else
                                calc.f_lat = 0;
                            end
                            calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2)^2);
                            calc.f_n = param.g*car.m+calc.f_down;
                            calc.f_max_long = sqrt( ((car.u_long*calc.f_n)^2) - ((((calc.f_lat)^2)*(car.u_long^2)) / ((car.u_lat)^2)) );

                            calc.f_max_motor = min(car.nm*car.t_max*car.gr/car.r_tire,...
                            (car.p_max_endurance*car.n_mech*car.n_elec/sim.state_p(2))*(1.5^(1/(sim.state_p(2)-car.v_max))));
                            calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d*sim.state_p(2)^2);
                            calc.f_cp_r = min(calc.f_max_long/2,calc.f_max_motor);

                            sim.state_c(1) = (calc.f_cp_r-calc.f_drag)/car.m;
                            sim.state_c(2) = sim.state_p(2) + sim.state_c(1)*sim.dt;
                            sim.state_c(3) = sim.state_p(3) + sim.state_c(2)*sim.dt;

                            sim.state_block_f(:,sim.state_index_f) = sim.state_c;
                            sim.power_block_f(sim.state_index_f) = (calc.f_cp_r*sim.state_c(2))/(car.n_mech*car.n_elec);

                            sim.state_index_f = sim.state_index_f + 1;
                            sim.state_p = sim.state_c;
                        end
                        sim.state_index_f = sim.state_index_f - 1;
                        %this is called if we need to perform braking to obide by
                        %the maximum section transfer speed.
                        if and(track.data(mod(track.track_i,length(track.data(:,1)))+1,3) ~= 0, sim.state_c(2) >= track.data(mod(track.track_i,length(track.data(:,1)))+1,3))
                            %we need to brake before entering the next track section
                            sim.state_c = [0;track.data(mod(track.track_i,length(track.data(:,1)))+1,3);track.data(track.track_i,1)];
                            sim.state_p = sim.state_c;

                            while sim.state_p(3) > 0
                                if track.data(track.track_i,2) ~= 0 %Calculate things for this timestep
                                    calc.f_lat = car.m*sim.state_c(2)^2/track.data(track.track_i,2);
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
                                sim.state_p(2) = sim.state_c(2) - sim.state_c(1)*sim.dt;
                                sim.state_p(3) = sim.state_c(3) - sim.state_c(2)*sim.dt;

                                sim.state_block_r(:,sim.state_index_r) = sim.state_c;

                                sim.state_index_r = sim.state_index_r + 1;
                                sim.state_c = sim.state_p;

                                while sim.state_p(3) <= sim.state_block_f(3,sim.state_index_f)
                                    sim.state_index_f = sim.state_index_f-1;
                                end
                                if sim.state_p(2) >= sim.state_block_f(2,sim.state_index_f)
                                    %found a match
                                    break;
                                end
                            end
                            %fix counter
                            sim.state_index_r = sim.state_index_r - 1;
                            %initialize next inital velocity
                            sim.state_p = [0;sim.state_block_r(2,1);0];
                            sim.state_c = sim.state_p;
                            %Update all logging in the braking case
                            log.endurance_times(track.track_i,lap_type) = (sim.state_index_f+sim.state_index_r)/1000;
                            log.endurance_power(track.track_i,lap_type) = sum(sim.power_block_f(1:sim.state_index_f))/(3600/sim.dt);
                            if and(log.plot == 1, lap_type == 1)
                                log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(:,1:sim.state_index_f);
                                log.lap_data(4,  log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(3,1:sim.state_index_f)+track.traversed;
                                log.lap_data(5,  log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.power_block_f(1:sim.state_index_f);
                                log.lap_data_index = log.lap_data_index+sim.state_index_f;
                                log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_r-1)) = fliplr(sim.state_block_r(:,1:sim.state_index_r));
                                log.lap_data(4,  log.lap_data_index:(log.lap_data_index+sim.state_index_r-1)) = fliplr(sim.state_block_r(3,1:sim.state_index_r))+track.traversed;
                                log.lap_data_index = log.lap_data_index+sim.state_index_r;
                            end
                        else %no braking was required before next segment
                            %initialize next inital velocity
                            sim.state_p = [0;sim.state_block_f(2,sim.state_index_f);0];
                            sim.state_c = sim.state_p;
                            %Update all logging in the non braking case
                            log.endurance_times(track.track_i,lap_type) = sim.state_index_f/1000;
                            log.endurance_power(track.track_i,lap_type) = sum(sim.power_block_f(1:sim.state_index_f))/(3600/sim.dt);
                            if and(log.plot == 1, lap_type == 1)
                                log.lap_data(1:3,log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(:,1:sim.state_index_f);
                                log.lap_data(4,  log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.state_block_f(3,1:sim.state_index_f)+track.traversed;
                                log.lap_data(5,  log.lap_data_index:(log.lap_data_index+sim.state_index_f-1)) = sim.power_block_f(1:sim.state_index_f);
                                log.lap_data_index = log.lap_data_index+sim.state_index_f;
                            end
                        end
                        %check to see if our work is no longer needed
                        if and(lap_type == 2, log.endurance_times(track.track_i,1)==log.endurance_times(track.track_i,2))
                            %Fill in what is needed for the full speed lap
                            log.endurance_times(track.track_i:length(track.data(:,1)),2) = log.endurance_times(track.track_i:length(track.data(:,1)),1);
                            log.endurance_power(track.track_i:length(track.data(:,1)),2) = log.endurance_power(track.track_i:length(track.data(:,1)),1);
                            break;
                        end
                        track.traversed = track.traversed + track.data(track.track_i,1);
                    end
                    %remove trailing zeros
                    if and(log.plot == 1, lap_type == 1)
                        log.lap_data = log.lap_data(:,1:log.lap_data_index-1);
                    end
                end
                %calculate metrics and fill the report in
                report.table(report.index,report.result_index+3) = sum(log.endurance_times(:,1))+sum(log.endurance_times(:,2))*(track.number_of_laps-1);
                report.table(report.index,report.result_index+4) = (sum(log.endurance_power(:,1))+sum(log.endurance_power(:,2))*(track.number_of_laps-1))/1000;

                disp([num2str(report.index),' of ',num2str(length(report.table(:,1)))]);
            end
        end
    end
end
report.table = real(report.table);


