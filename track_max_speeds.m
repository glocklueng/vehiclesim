function [ solved_track ] = track_max_speeds( car, param, tracks, sim )
%TRACK_MAX_SPEEDS solve the maximum velocity for each track segment
%Need to back calculate the max input segment speed for the
%whole track. Start by looking at the max speed according to
%radius of curvature, then make sure that those speed can be
%reached by reverse brakinging from each segment backwards
%around the track until the steady state solution is found

syms r v;

f_lat = car.swd * centripetal_force(car, v, r);
f_long = drag_force(car, param, v); %for now assuming
f_norm = static_weight_rear(car, param) + ...
         down_force_rear(car, param, v) + ...
         weight_transfer(car, f_long);

eqn = traction_ellipse(car, f_lat, f_long, f_norm, nonideal.corner_traction);
eqn = solve(eqn,v);

t_len = length(tracks); %cause i call it so often, safe a few strokes
% calculate the max velocity due to cornering
for i = 1:t_len

    if tracks(i).type ~= track_segment.straight
        vel = max(real(double(subs(eqn, r, tracks(i).radius ))));
    else
        vel = Inf; %in the straight line, there is NO LIMIT!
    end
    
    tracks(i).max_vel = vel;
    tracks(i).max_vel_radius = vel; %just for comparing later on

end

% iterate and calculate max velocity if you are trying to break
% start one before the end, because the end doesn't depend on the next
% track
for i = (t_len-1):-1:1
    i_c = wrapN(i  , t_len); % one based modulo
    i_n = wrapN(i+1, t_len);
    v_max_c = tracks(i_c).max_vel; %current velocity
    v_max_n = tracks(i_n).max_vel; %next velocity
    
    %if this segment is behind a restricted section and
    %it is straight or has a required speed greater than the
    %future speed, then we need to update
    if v_max_n < v_max_c
        %the state at the end of the track segment
        % not accelerating, the velocity is the max velocity of the next
        % segment, and the position is at the end of the track
        s_c.acc = 0;
        s_c.vel = v_max_n;
        s_c.pos = tracks(i_c).arc_length;
        
        s_p = s_c; %we will change one state and update it
        
        while s_p.pos > 0
            
            f_lat = centripetal_force(car, s_c.vel, tracks(i_c).radius);
            f_n = static_weight(car, param) + ...
                  down_force_total(car, param, s_c.vel);
            
            % rearange the ellipis equation to solve for maximum f_long
            f_long_max = sqrt((car.u_long*f_n)^2 - (f_lat*car.u_long/car.u_lat)^2);
            f_drag = drag_force(car, param, s_c.vel);
            
            %this can be ode45'ed or something similar in the future
            %calculate the dt previous state
            s_p.acc = (-f_long_max-f_drag)/car.m;
            s_p.vel = s_c.vel - s_p.acc*sim.dt;
            s_p.pos = s_c.pos - s_p.vel*sim.dt;
            
            % update the current state with what we calculated
            s_c = s_p;
        end
        %update our required speed for the entrence of this segment
        %to ensure that we can brake for the next one.
        %i have a feeling that s_c will never be larger than max_vel, cause
        %if it is, then while calculating the friction on the tire we
        %messed up
        tracks(i_c).max_vel = min(tracks(i_c).max_vel, s_c.vel);
        
    end
    
end

%since tracks is a copy
solved_track = tracks;
end

