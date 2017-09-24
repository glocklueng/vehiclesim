classdef track_segment
    %TRACK_SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        left = -1;
        straight = 0;
        right = 1; 
    end
    properties
        %assigned values
        type;
        arc_length;
        radius;
        
        %calculated values
        max_vel_radius %due to radius
        max_vel %due to the shape of the segments in front and braking
        
        %state array for braking, and then just driving though the track
        state_brake;
        state_drive;
    end
    
    methods
        function obj = track_segment(track_array)
            if nargin ~= 0
                l = size(track_array,1);
                obj(l) = track_segment;
                for i = 1:l
                    obj(i).type = track_array(i, 1);
                    obj(i).arc_length = track_array(i, 2);
                    obj(i).radius = track_array(i, 3);
                    if obj(i).type == 0
                        obj(i).radius = Inf;
                    end
                end
            end
        end
    end
    
end

