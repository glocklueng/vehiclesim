function [ ] = plot_track_rework( tracks )
%PLOT_TRACK_REWORK Summary of this function goes here


%% plot the track data

left = -1;
straight = 0;
right = 1;

figure
hold on

dir = [0, 1];

pos_cur = [0, 0];
pos_next = [0, 0];

for i=1:length(tracks)
    %type, arc_length, radius = track.input(i,:); if only it was python...
    type        = tracks(i).type;
    arc_length  = tracks(i).arc_length;
    radius      = tracks(i).radius;
    
    %normalize dir just in case
    dir = dir / norm(dir);
    
    %straight line
    if type == straight
        pos_next = pos_cur + dir * arc_length;
        line([pos_cur(1), pos_next(1)], [pos_cur(2), pos_next(2)]);
    else
        % perpendicular is needed to find radius
        perp = dir(:,[2,1]); % points toward the center
        %it depends if we are turning left or right
        if type == left
            perp(1) = -perp(1);
        else
            perp(2) = -perp(2);
        end
        
        theta = arc_length / radius;%angle made while turning
        if type == left
            theta = -theta;
        end
        
        %rotation matrix to apply
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        pos_next = pos_cur + perp * radius - perp * radius * R;

        ppp = pos_cur;
        ppn = pos_cur;
        % plot as a line right now
        for t=linspace(0, theta)
            %rotation matrix to apply
            R = [cos(t) -sin(t); sin(t) cos(t)];
            ppn = pos_cur + perp * radius - perp * radius * R;
            line([ppp(1), ppn(1)], [ppp(2), ppn(2)]);
            ppp = ppn;
        end
        
        % direction after the arc circle
        dir = dir * R;
    end
    
    %update the current position
    pos_cur = pos_next;
end
end

