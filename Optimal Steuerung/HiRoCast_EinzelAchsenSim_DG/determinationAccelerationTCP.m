function [orientation_xy] = determinationAccelerationTCP(optimized_translational_values)
    position_TCP = [];
    
    % Transformation der Achsen in TCP Koordinaten
    for e = 1:length(optimized_translational_values)
        [pos2, eulerZYX2,~, ~] = vorwaertskinematik(optimized_translational_values(e, 2:7));          
        eulZYX(e,1:3) = transpose(eulerZYX2);
        position_TCP(end+1,1:3) = transpose(pos2(1:3,1));
    end  
   
    [euler_XY,~,~,~,time_Spline,~] = splineOptimal(eulZYX(:,1),optimized_translational_values(1:end-1,1),false);   
    
    for i = 1:3
        [~,~,acceleration,~,~,~] = splineOptimal(position_TCP(:,i),optimized_translational_values(1:end-1,1),false);           
        acceleration_xy(:,i) = acceleration;
    end
  
    %Rotation um Z
    for i = 1:size(euler_XY,2)
        acceleration_xy_TCP(i,1:3) = transpose(RotationDegUmZ(-euler_XY(i))*transpose(acceleration_xy(i,1:3)));          
    end
   
    %Umorienterung um schwappen durch beschleunigung zu Kompensieren
    for t = 1:length(acceleration_xy_TCP)
       orientation_xy(t,1) = time_Spline(1,t);
       orientation_xy(t,2) = atand(acceleration_xy_TCP(t,1)/(9.81+acceleration_xy_TCP(t,3))); % TODO: für 9.81 die Z Komponente mit rein bringen
       orientation_xy(t,3) = atand(acceleration_xy_TCP(t,2)/(9.81+acceleration_xy_TCP(t,3)));   
    end

    figure
    hold on
    plot(time_Spline,acceleration_xy_TCP(:,1))
    plot(time_Spline,acceleration_xy_TCP(:,2))
    plot(time_Spline,acceleration_xy_TCP(:,3))
    legend('Acc_X_TCP','Acc_Y_TCP','Acc_Z_TCP')
    figure;
    hold on
%     plot(time_Spline,orientation_xy(:,1))
    plot(time_Spline,orientation_xy(:,2))
    plot(time_Spline,orientation_xy(:,3))
    legend('orientation_xy_X_TCP','orientation_xy_Y_TCP','orientation_xy_Z_TCP')
end

