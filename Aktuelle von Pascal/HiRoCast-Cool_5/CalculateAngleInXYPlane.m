function [angle] = CalculateAngleInXYPlane(preferredDirection, otherVector)
    % Calculates the angle between the preferred direction and an arbitrary vector, not considering the zComponent of the latter
    
    %project other vector on XYPlane by removing z-Component
    otherVector(3,1) = 0;
    otherVector = otherVector/norm(otherVector);
    preferredDirection = preferredDirection/norm(preferredDirection);
    
    % calculate angle of both vectors in XYPlane   
    angle_3_soll = atan2d(preferredDirection(2,1),preferredDirection(1,1));
    angle_3_ist = atan2d(otherVector(2,1),otherVector(1,1));

    angle = angle_3_ist-angle_3_soll;

    if angle > 180
        angle = -180+abs(angle-180);
    elseif angle < -180
        angle = 180-abs(angle+180);
    end

%altanative gleichwertige lösungen       
%  preferredDirection_norm = norm(preferredDirection);
%  otherVector_norm = norm(otherVector);   
%  angle_2 = acosd(dot(preferredDirection,otherVector)/preferredDirection_norm*otherVector_norm);

%alte Version, die weniger anschaulich ist
% % % %  angle_ = atan2d(norm(cross(preferredDirection,otherVector)), dot(preferredDirection,otherVector));       
% % % %     % calculate sign of the angle
% % % %     crossproduct = cross(preferredDirection, otherVector);
% % % %     normalVectorOfXYPlane = [0, 0, 1];
% % % %     
% % % %     if (dot(normalVectorOfXYPlane, crossproduct) < 0)
% % % %         angle_ = -angle_;
% % % %     end

end
