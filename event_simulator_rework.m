
%%  some params
%clear;
syms r v;

sim.dt = 0.001;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

nonideal.corner_traction = 1;
nonideal.uniform_braking = 1;

%% load track data
track_data

%% load the governing equations
governing_equations

%% log

log.plot = 1;
log.lap_data = zeros(5,200000);
log.accel_plot = zeros(3,20/sim.dt);
log.autocross_times = zeros(length(track.input(:,1)),1);
log.endurance_times = zeros(length(track.input(:,1)),2);
log.endurance_power = zeros(length(track.input(:,1)),2);

%% define the car models
mass = 320;%[200,220,240,260,280];
center_gravity = 0.3;
wheel_base = 1.6;
static_weight_distribution = 0.6; %higher is more rearward
center_pressure = 0.5;
tire_friction_longitudinal = 1.6;%[1.7,1.1,1.2,1.3,1.4,1.5,1.6,1.7];
tire_friction_lateral = 1.5;
tire_radius = 0.265;
aero_af = 1.24;
aero_c_downforce = 1.61;
aero_c_drag = 0.96;
efficiency_mechanical = 0.95;
efficiency_electrical = 0.92;
power_max = 80000;
power_max_endurace_limit = 3000;%[30000,35000,40000,45000,50000,55000,60000,65000,70000,75000,80000];
motor_number = 2;
motor_torque_max = 90; %actually 100
motor_speed_max = 7000;
gear_ratio = 5.5;%[2,2.25,2.5,2.75,3,3.25,3.5,3.75,4,4.25,4.5,4.75,5,5.25,5.5,5.75,6,6.25,6.5];

cars = generate_cars(mass,...
                center_gravity,...
                wheel_base,...
                static_weight_distribution,...
                center_pressure,...
                tire_friction_longitudinal,...
                tire_friction_lateral,...
                tire_radius,...
                aero_af,...
                aero_c_downforce,...
                aero_c_drag,...
                efficiency_mechanical,...
                efficiency_electrical,...
                power_max,...
                power_max_endurace_limit,...
                motor_number,...
                motor_torque_max,...
                motor_speed_max,...
                gear_ratio);


%% simulation
report.result_index = 0; %amount of vector data
report.table = zeros(length(cars),report.result_index+4);
report.index = 0;
for car_i = 1:length(cars)
    car = cars(car_i);

    %Set and record parameters
    report.index = report.index + 1;

    %should be generalized with the rest of the track event
%     %==============================================================
%     %Acceleration Event
%     %==============================================================
%     sim.state_p = [0;0;0];%initialize to zero speed
%     sim.state_index_f = 1;
%     sim.state_c = sim.state_p;
% 
%     while sim.state_c(3) < 75
% 
%         calc.f_down  = (0.5*param.air_p*car.a_f*car.c_df*sim.state_p(2)^2);
%         calc.f_drag  = (0.5*param.air_p*car.a_f*car.c_d *sim.state_p(2)^2);
%         calc.wt = car.m*sim.state_p(1)*car.cg/car.wb;
%         calc.f_frict_f = car.u_long*(param.g*car.m*(1-car.swd)+calc.f_down*(1-car.cp)-calc.wt);
%         calc.f_frict_r = car.u_long*(param.g*car.m*(car.swd)+calc.f_down*(car.cp)+calc.wt);
%         calc.f_max_motor = min(car.nm*car.t_max*car.gr/car.r_tire,...
%         (car.p_max*car.n_mech*car.n_elec/sim.state_p(2))*(1.5^(1/(sim.state_p(2)-car.v_max))));
%         calc.f_cp_r = min(calc.f_frict_r,calc.f_max_motor);
% 
%         sim.state_c(1) = (calc.f_cp_r-calc.f_drag)/car.m;
%         sim.state_c(2) = sim.state_p(2) + sim.state_c(1)*sim.dt;
%         sim.state_c(3) = sim.state_p(3) + sim.state_c(2)*sim.dt;
%         if log.plot == 1
%             log.accel_data(:,sim.state_index_f) = sim.state_c;
%             sim.state_index_f = sim.state_index_f + 1;
%         end
%         sim.state_p = sim.state_c;
%     end
%     report.table(report.index,report.result_index+1) = sim.state_index_f/1000;


    %==============================================================
    %Autocross and Endurance, calculate track
    %==============================================================
    %Need to back calculate the max input segment speed for the
    %whole track. Start by looking at the max speed according to
    %radius of curvature, then make sure that those speed can be
    %reached by reverse brakinging from each segment backwards
    %around the track until the steady state solution is found
    f_lat = car.swd * centripetal_force(car, v, r);
    f_long = drag_force(car, param, v); %for now assuming
    f_norm = static_weight_rear(car, param) + ...
             down_force_rear(car, param, v) + ...
             weight_transfer(car, f_long);
         
    eqn = traction_ellipse(car, f_lat, f_long, f_norm, nonideal.corner_traction);
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

    disp([num2str(report.index),' of ',num2str(length(cars))]);
end

report.table = real(report.table);


