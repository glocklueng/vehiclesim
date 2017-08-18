

clear x y speed data;
close all

data = log.lap_data;
trackPoints = 10000;
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

trackLength = sum(track.input(:,2));
lenthIncrement =  trackLength / trackPoints ;
x = 0;
y = 0;
currentBearing = 0;

lengthTravledTotal = 0;
lengthTravledSegment = 0;

trackSegment = 1;

for i = 2:trackPoints
    
    lastPos = [x(i-1);y(i-1)];
    
    lengthTravledTotal = lengthTravledTotal + lenthIncrement;
    lengthTravledSegment = lengthTravledSegment + lenthIncrement;
    displacment(i) = lengthTravledTotal;
    
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
    [accuracy index] = min(abs(data(4,:)-displacment(i)));
    speed(i) = data(2,index);
    
   
end

colormap jet;

scatter(x,y,50,speed)



