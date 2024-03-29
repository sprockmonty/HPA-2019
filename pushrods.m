clear
clc
close all

%define stick angle
stickAngle = 4; %in degrees from north, clockwise positive

%%	Start program

stickAngle = deg2rad(stickAngle);

%define hinge locations
hinges = [0, 0; 650, 213.5];

%each hinge has two rods, an upper rod and a lower rod, if radius = 0, there no rod is present. Upper rod is measured in positive radius, lower is measured in negative radius.
%rods have indexing, 1 = r1c1, 2 = r1c2, 3 = r2c1, 4 = r2c2 etc.
%rod 1 must be the stick
hingeRodsRadi = [50, 0; 0,-150];





%link hinge rods to additional rods in matrix mx3, where first column is rod length, second is index of first hinge rod to connect to, third is index of second hinge rod to connect to.
%tops are odd, bottoms are even for consistency
linkRods = [647, 1, 4];

if sum(not(size(hinges) == size(hingeRodsRadi)) | not(size(linkRods,1) == size(hinges,1) - 1))
	error('number of rodes is not consistent with number of hinges')
end

%calculate angles
theta = zeros(1, size(hinges,1));
theta(1) = stickAngle;
figure
hold on
daspect([1 1 1])
for i = 1:size(hinges,1)-1
    hinge1 = (mod(linkRods(i,2),2) + linkRods(i,2)) / 2;
    hinge2 = (mod(linkRods(i,3),2) + linkRods(i,3)) / 2;
    hinge1Loc = hinges(hinge1,:);
    hinge2Loc = hinges(hinge2,:);
    hinge1Radi = hingeRodsRadi(hinge1, mod(linkRods(i,2)+1,2)+1);
    hinge2Radi = hingeRodsRadi(hinge2, mod(linkRods(i,3)+1,2)+1);
    point1 = [hinge1Radi * sin(theta(i)) + hinge1Loc(1),...
        hinge1Radi * cos(theta(i)) + hinge1Loc(2)];
    p1h2Dist = pdist([point1;hinge2Loc], 'euclidean');
    thetaX = atan2(hinge2Loc(1) - point1(1), hinge2Loc(2) - point1(2));
    thetaY = acos((hinge2Radi^2 + p1h2Dist^2 - linkRods(i,1)^2 )/...
        (2*hinge2Radi*p1h2Dist));
    theta(i+1) = thetaY -pi + thetaX;
    point2 = [hinge2Radi * sin(theta(i+1)) + hinge2Loc(1),...
        hinge2Radi * cos(theta(i+1)) + hinge2Loc(2)];
    scatter(hinge1Loc(1),hinge1Loc(2),'b')
    scatter(hinge2Loc(1), hinge2Loc(2),'b')
    xToPlot = [hinge1Loc(1), point1(1), point2(1), hinge2Loc(1)];
    yToPlot = [hinge1Loc(2), point1(2), point2(2), hinge2Loc(2)];
    plot(xToPlot, yToPlot, 'r')
end

%plot all hinges again just incase any were missed
for i = 1:size(hinges,1)
    xToPlot = [hinges(i,1) + sin(theta(i))*hingeRodsRadi(i,1), ...
        hinges(i,1) + sin(theta(i))*hingeRodsRadi(i,2)];
    yToPlot = [hinges(i,2) + cos(theta(i))*hingeRodsRadi(i,1), ...
        hinges(i,2) + cos(theta(i))*hingeRodsRadi(i,2)];
    plot(xToPlot, yToPlot, 'r')
end



% output surface angles
finalTheta = rad2deg(theta(end));
if finalTheta < 0
    finalTheta = finalTheta + 360;
end
    
if finalTheta <90 || finalTheta <=270 && finalTheta >180
    thetaSurface = mod(finalTheta, 90);
else
    thetaSurface = -90+mod(finalTheta, 90);
end
fprintf("The output angle is %f degrees\n", finalTheta)
fprintf("The normal (surface deflection) angle is %f degrees", thetaSurface)

angles = deflecAngleCalc(hinges, hingeRodsRadi, linkRods, [-21, 29]);
figure
plot(angles(:,1), angles(:,2))
xlabel('stick input')
ylabel('elevator deflection')

