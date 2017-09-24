
%%  some params
%clear;
sim.dt = 0.001;
param.g = 9.81;
param.air_p = 1.15; %in lincoln

%should be removed, as the rework doesn properly support it yet
nonideal.corner_traction = 1;
nonideal.uniform_braking = 1;

%% load track data
track_data

%% load the governing equations
governing_equations

%% log
% not sure how it works, will maybe redone later

% log.plot = 1;
% log.lap_data = zeros(5,200000);
% log.accel_plot = zeros(3,20/sim.dt);
% log.autocross_times = zeros(length(track.input(:,1)),1);
% log.endurance_times = zeros(length(track.input(:,1)),2);
% log.endurance_power = zeros(length(track.input(:,1)),2);

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
    
    %brake backwards and find the max speeds
    autotrack = track_max_speeds( car, param, autotrack, sim );
    
    %now drive forward through the track

    disp([num2str(report.index),' of ',num2str(length(cars))]);
end

report.table = real(report.table);


