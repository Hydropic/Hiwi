function [x] = backwardTransformationRoboDK(Position_xyz, timeLine, splineDiscretization, x_xyz, axesPointConfigs,eulerZYX)

    for u = 1: length(Position_xyz)

                  
        angleTCPxyz = [(eulerZYX(u,1)), (eulerZYX(u,2)), (eulerZYX(u,3))]; 
        xyzrpw(:,u) = [1000*Position_xyz(u, 1), 1000*Position_xyz(u, 2), 1000*Position_xyz(u, 3), angleTCPxyz(1), angleTCPxyz(2), angleTCPxyz(3)];        
    end

    addpath('C:\RoboDK\Matlab');

    % Ermittlung der Roboterposen in RoboDK
    [joints] = HiRoCast_rueckweartstransformation(xyzrpw);

    show_joint2 = [];

    show_joint2(1:splineDiscretization,1) = timeLine(1, 2);
    show_joint2(:,2) = deg2rad(joints(:,1));
    show_joint2(:,3) = deg2rad(joints(:,2));
    show_joint2(:,4) = deg2rad(joints(:,3));
    show_joint2(:,5) = deg2rad(joints(:,4));
    show_joint2(:,6) = deg2rad(joints(:,5));
    show_joint2(:,7) = deg2rad(joints(:,6));

    show_spline(show_joint2,'as')
    x = show_joint2;

    stop = 0;
end

