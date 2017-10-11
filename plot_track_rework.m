function [ ] = plot_track_rework( tracks, param )
%PLOT_TRACK_REWORK Summary of this function goes here


% plot the track data

left = -1;
straight = 0;
right = 1;

figure
hold on
pbaspect([1 1 1])

dir = [0, 1];

pos_cur = [0, 0];
pos_next = [0, 0];

%get maximum value of param
max_val = 0; %hardcoded for now
for i=1:length(tracks)
    track = tracks(i);
    s = track.state_drive;
    val = max([s.(param)]);
    max_val = max(max_val, abs(val));
end

title(strcat('Track with:', param, '. Max val: ', num2str(max_val)));

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
        
        %plot the segment
        ppp = pos_cur;
        ppn = pos_cur;
        
        % plot as a line right now
        %for t=linspace(0, theta)
        for j=1:10:length(tracks(i).state_drive)
            cur_state = tracks(i).state_drive(j);
            
            %color based on breaking / acceleration
            color = gen_color(cur_state, param, max_val);
            
            %rotation matrix to apply
            ppn = pos_cur + dir*cur_state.pos;
            line([ppp(1), ppn(1)], [ppp(2), ppn(2)], 'Color', color, 'LineWidth', 2.0);
            ppp = ppn;
        end
        
        
        %line([pos_cur(1), pos_next(1)], [pos_cur(2), pos_next(2)]);
    else
        % perpendicular is needed to find radius
        perp = dir(:, [2,1]); % points toward the center
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
        %the end position of the track segment
        pos_next = pos_cur + perp * radius - perp * radius * R;

        %plot the segment
        ppp = pos_cur;
        ppn = pos_cur;
        % plot as a line right now
        %for t=linspace(0, theta)
        for j=1:10:length(tracks(i).state_drive)
            cur_state = tracks(i).state_drive(j);
            
            %the angle based on where we are
            t = sign(theta)*cur_state.pos / tracks(i).radius;
            
            %color based on breaking / acceleration
            color = gen_color(cur_state, param, max_val);
            
            %rotation matrix to apply
            R = [cos(t) -sin(t); sin(t) cos(t)];
            ppn = pos_cur + perp * radius - perp * radius * R;
            line([ppp(1), ppn(1)], [ppp(2), ppn(2)], 'Color', color, 'LineWidth', 2.0);
            ppp = ppn;
        end
        
        % direction after the arc circle
        dir = dir * R;
    end
    
    %update the current position
    pos_cur = pos_next;
end
end

