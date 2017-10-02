function [ s_n ] = motion_sim( car, param, sim, track, dir, s_c )
%MOTION_SIM simulate the car either driving forwards, or
% have the car brake back from the end of the track
% car - car object
% param - param object
% sim - sim object
% track - the track segement the car is on
% dir - -1 is breaking, +1 is going forwards
% vel_init - initial velocity (either the start or the end of the track)

%the state is updated by dt and returned

governing_equations; %load those functions

%calculate the next times step state
% this is done after the update, because if it goes out of bounds the
% check happens after (the while loop check), and then it just exits
f_lat = centripetal_force(car, s_c.vel, track.radius);
f_n = static_weight_total(car, param) + ...
      down_force_total(car, param, s_c.vel);

% rearange the ellipis equation to solve for maximum f_long
f_long_max = sqrt((car.u_long*f_n)^2 - (f_lat*car.u_long/car.u_lat)^2);
f_drag = drag_force(car, param, s_c.vel);

%this can be ode45'ed or something similar in the future
%calculate the dt previous state
s_n.acc = (dir*f_long_max - f_drag)/car.m;
s_n.vel = s_c.vel + dir*s_n.acc*sim.dt;
if s_n.vel > track.max_vel
    %this means that next iteration is screwed cause our acceleration was
    %too hard
    s_n.vel = track.max_vel; %we are limited to this speed
    
    %calculate the acceleration that would get us to that speed
    %that acceleration is lower than the one we just calculated
    s_n.acc = (s_n.vel - s_c.vel)/(dir*sim.dt);
end

s_n.pos = s_c.pos + dir*s_n.vel*sim.dt;
 

end

