classdef car_consts
    %CAR_CONSTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m;
        cg;
        wb;
        swd;
        cp;
        
        u_long;
        u_lat;
        r_tire;
        %aero
        a_f;
        c_df;
        c_d;
        %motor and gearbox
        n_mech;
        n_elec;
        p_max;
        p_max_endurance;
        
        nm;
        t_max;
        n_max;
        
        gr;
        v_max;
    end
    
    methods
        %TODO convert this funtion to return the object array instead
        function obj = car_consts(mass,...
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
            %so that object arrays can happen
            if nargin > 0
                obj.m = mass;
                obj.cg = center_gravity;
                obj.wb = wheel_base;
                obj.swd = static_weight_distribution;
                obj.cp = center_pressure;
                obj.u_long = tire_friction_longitudinal;
                obj.u_lat = tire_friction_lateral;
                obj.r_tire = tire_radius;
                obj.a_f = aero_af;
                obj.c_df = aero_c_downforce;
                obj.c_d = aero_c_drag;
                obj.n_mech = efficiency_mechanical;
                obj.n_elec = efficiency_electrical;
                obj.p_max = power_max;
                obj.p_max_endurance = power_max_endurace_limit;
                obj.nm = motor_number;
                obj.t_max = motor_torque_max;
                obj.n_max = motor_speed_max;
                obj.gr = gear_ratio;
                obj.v_max = obj.n_max*2*pi*obj.r_tire/(60*obj.gr);
            end
            
        end
    end
    
end

