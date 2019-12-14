function [thetaOut] = deflecAngleCalc(hinges, hingeRodsRadi, linkRods, range)
%deflecAngleCalc Calculate output angles for a given range
%range in degrees in format [start, end]
    range = deg2rad(range);
    resolution = 100;
    step = (range(2) - range(1)) / resolution;
    thetaOut = [];
    for j = 1:resolution
        theta = zeros(1, size(hinges,1));
        theta(1) = (j-1)*(step) + range(1);
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
        end
        
        finalTheta = rad2deg(theta(end));
        if finalTheta < 0
            finalTheta = finalTheta + 360;
        end

        if finalTheta <90 || finalTheta <=270 && finalTheta >180
            thetaSurface = mod(finalTheta, 90);
        else
            thetaSurface = -90+mod(finalTheta, 90);
        end
        thetaOut = [thetaOut; rad2deg(theta(1)), thetaSurface];
    end
end

