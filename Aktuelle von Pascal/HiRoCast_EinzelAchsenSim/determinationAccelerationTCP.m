function [orientation_xy] = determinationAccelerationTCP(optimized_translational_values)
    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, 1);
    for i = 2:4
        [achsstellung,vilocity,acceleration,jerk,time,splinePunkt] = splineOptimal(position_TCP(:,i),position_TCP(1:end-1,1),false);
        accEnd = acceleration(end);
        timeEnd = time(end);
% %         % Ende auf 0 setzen
% %         numExtensionPoints = 5
% %         for e = 1:numExtensionPoints            
% %             acceleration(end+1) = accEnd - accEnd*(e/numExtensionPoints);
% %             time(end+1) = time(end)+0.08;
% %         end
        
        if i == 2
            acceleration_xy(:,1) = time;
        end
        acceleration_xy(:,i) = acceleration;
    end
%     figure
%     plot(acceleration_xy(:,1), acceleration_xy(:,4))
%     hold on;
%     plot(acceleration_xy(:,1), acceleration_xy(:,3))
    for t = 1:length(acceleration_xy)
        orientation_xy(t,1) = acceleration_xy(t,1);
        orientation_xy(t,2) = rad2deg(atan(acceleration_xy(t,2)/(9.81+acceleration_xy(t,4))));
        % orientation_xy(t,3) = 0;
        orientation_xy(t,3) = 1*rad2deg(atan(acceleration_xy(t,3)/(9.81+acceleration_xy(t,4))));
    end

%     plot(acceleration_xy)
%     figure;
%     plot(orientation_xy)
end

