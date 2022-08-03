function [angle] = CalculateAngleInXYPlane(preferredDirection, otherVector)
% Calculates the angle between the preferred direction and an arbitrary vector, not considering the zComponent of the latter

%project other vector on XYPlane by removing z-Component
otherVector(3) = 0;

% calculate angle of both vectors in XYPlane
angle = atan2d(norm(cross(preferredDirection,otherVector)), dot(preferredDirection,otherVector));

% calculate sign of the angle
crossproduct = cross(preferredDirection, otherVector);
normalVectorOfXYPlane = [0, 0, 1];

if (dot(normalVectorOfXYPlane, crossproduct) < 0)
    angle = -angle;
end
