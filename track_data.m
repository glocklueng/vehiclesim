
%% track data for simulator
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

autotrack = track_segment(track.input);

%% return the script, so not to plot graphs normally
return

%% plot the track data

left = -1;
straight = 0;
right = 1;

figure
hold on

dir = [0, 1];

pos_cur = [0, 0];
pos_next = [0, 0];

for i=1:length(track.input)
    %type, arc_length, radius = track.input(i,:); if only it was python...
    type        = track.input(i,1);
    arc_length  = track.input(i,2);
    radius      = track.input(i,3);
    
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
        %line([pos_cur(1), pos_next(1)], [pos_cur(2), pos_next(2)]);
        
        % direction after the arc circle
        dir = dir * R;
    end
    
    %update the current position
    pos_cur = pos_next;
end
