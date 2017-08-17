clear all
close all

trackPoints = 1000;
track.input = [
    1 , 20 ,10;
    0 ,30 ,0;
    -1 , 10 ,20;
    0, 40 ,0;
    1, 8  ,8;
    -1,6  ,120;
    0, 50 ,0];

trackLength = sum(track.input(:,2));
lenthIncrement =  trackLength / trackPoints ;
x = 0;
y = 0;
currentBearing = 0;

lengthTravledTotal = 0;
lengthTravledSegment = 0;

trackSegment = 1;

for i = 2:trackPoints
    
    lastPos = [x(i-1);y(i-1)]
    
    lengthTravledTotal = lengthTravledTotal + lenthIncrement;
    lengthTravledSegment = lengthTravledSegment + lenthIncrement;
    
    if lengthTravledSegment> track.input(trackSegment,2)
        lengthTravledSegment = lengthTravledSegment - track.input(trackSegment,2);
        trackSegment = trackSegment +1
        line(x,y)
    end
    
    travel = [0;lenthIncrement];
    
    if track.input(trackSegment,1) ~= 0
        angle = 360*(lenthIncrement/(pi * 2 * track.input(trackSegment,3)));
        forwardTravel  = sind(angle)*track.input(trackSegment,3);
        transverseTravel = track.input(trackSegment,3) - cosd(angle)*track.input(trackSegment,3);
        if track.input(trackSegment,1) == 1 %left
            transverseTravel = - transverseTravel;
        end
        travel = [transverseTravel;forwardTravel];
    end
    
    travel = [cosd(currentBearing),-sind(currentBearing);sind(currentBearing),cosd(currentBearing)]*travel;
    newpos = travel+lastPos;
    x(i) = newpos(1);
    y(i) = newpos(2);
    
    if track.input(trackSegment,1) == -1 %Right
        currentBearing = currentBearing + angle;
    else if track.input(trackSegment,1) == 1
            currentBearing = currentBearing - angle;
        end
    end
    
    
end







