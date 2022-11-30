function [orientation_xy] = determinationAccelerationTCP(optimized_translational_values)
    position_TCP = [];
    % Transformation der Achsen in TCP Koordinaten
    for e = 1:length(optimized_translational_values)
        [pos2, eulerZYX2,eulerXYZ2, y_direction2] = vorwaertskinematik(optimized_translational_values(e, 2:7));  

        position_TCP(end+1,1) = optimized_translational_values(e, 1);
        position_TCP(end,2) = pos2(1);
        position_TCP(end,3) = pos2(2);
        position_TCP(end,4) = pos2(3);
    end    
    
    for i = 2:4
        [achsstellung,vilocity,acceleration,jerk,time,splinePunkt] = splineOptimal(position_TCP(:,i),position_TCP(1:end-1,1),false);
        accEnd = acceleration(end);
        timeEnd = time(end);        
        if i == 2
            acceleration_xy(:,1) = time;
        end
        acceleration_xy(:,i) = acceleration;
    end

    for t = 1:length(acceleration_xy)
        orientation_xy(t,1) = acceleration_xy(t,1);
        orientation_xy(t,2) = rad2deg(atan(acceleration_xy(t,2)/(9.81+acceleration_xy(t,4)))); % TODO: für 9.81 die Z Komponente mit rein bringen
        % orientation_xy(t,3) = 0;
        orientation_xy(t,3) = rad2deg(atan(acceleration_xy(t,3)/(9.81+acceleration_xy(t,4))));
    end

%     plot(acceleration_xy)
%     figure;
%     plot(orientation_xy)
end

