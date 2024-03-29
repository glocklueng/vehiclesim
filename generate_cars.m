function [ cars ] = generate_cars(mass,...
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
                gear_ratio)

 %cause i'm a dirty bastard and you know it
combs = allcomb(mass,...
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
%combs is a P x N, where N is the number of arguments for the car
% and P is the amount of combinations we have


cars(size(combs,1)) = car_consts;

for i = 1:length(cars)
    temp = combs(i,:); %the current car combination
    args = num2cell(temp); %convert to cell array
    cars(i) = car_consts(args{:}); %just so i can auto pass it into here
end

end

