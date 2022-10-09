function [Position_xyz, timeLine] = generateTCPPath(optimization_values, wayPoints, splineDiscretization, visualizeTCPPath, min_values, max_values, jerkBoundaries)

    tvec = 0:0.01:optimization_values(1, 3);
    tpts = optimization_values(1, 1:3);

    VelocityBoundaryCondition_x = [0 optimization_values(2, 1) 0]
    VelocityBoundaryCondition_y = [0 optimization_values(2, 2) 0]
    VelocityBoundaryCondition_z = [0 optimization_values(2, 3) 0]

    AccelerationBoundaryCondition_x = [0 optimization_values(3, 1) 0]
    AccelerationBoundaryCondition_y = [0 optimization_values(3, 2) 0]
    AccelerationBoundaryCondition_z = [0 optimization_values(3, 3) 0]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)
    [q_y,qd_y,qdd_y,pp_y] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y)
    [q_z,qd_z,qdd_z,pp_z] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_z,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_z)

    q_xyz = [transpose(q_x), transpose(q_y), transpose(q_z)];
    qd_xyz = [transpose(qd_x), transpose(qd_y), transpose(qd_z)];
    qdd_xyz = [transpose(qdd_x), transpose(qdd_y), transpose(qdd_z)];
    pp_xyz = [transpose(pp_x), transpose(pp_y), transpose(pp_z)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);

    lin_xx_x_y = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), lin_xx_x_y);

    lin_xx_x_z = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_z = spline(tvec(1, :), q_xyz(:, 3), lin_xx_x_z);

    max_spline_d_x = max(qd_xyz(:,1))
    max_spline_d_y = max(qd_xyz(:,2))
    max_spline_d_z = max(qd_xyz(:,3))

    min_spline_d_x = min(qd_xyz(:,1))
    min_spline_d_y = min(qd_xyz(:,2))
    min_spline_d_z = min(qd_xyz(:,3))

    max_spline_dd_x = max(qdd_xyz(:,1))
    max_spline_dd_y = max(qdd_xyz(:,2))
    max_spline_dd_z = max(qdd_xyz(:,3))

    min_spline_dd_x = min(qdd_xyz(:,1))
    min_spline_dd_y = min(qdd_xyz(:,2))
    min_spline_dd_z = min(qdd_xyz(:,3))

    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        xline(tpts(2))
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        xline(tpts(2))
        yline(max_values(2,1), 'b')
        yline(min_values(2,1), 'b')
        yline(max_values(2,2), 'r')
        yline(min_values(2,2), 'r')
        yline(max_values(2,3), 'y')
        yline(min_values(2,3), 'y')
        legend('X','Y','Z') 

        subplot(4,1,3)        
        plot(tvec, qdd_xyz)
        xlabel('t')
        ylabel('acceleration')
        xline(tpts(2))
        yline(max_values(3,1), 'b')
        yline(min_values(3,1), 'b')
        yline(max_values(3,2), 'r')
        yline(min_values(3,2), 'r')
        yline(max_values(3,3), 'y')
        yline(min_values(3,3), 'y')
        legend('X','Y','Z') 

        qddd_xyz = diff(qdd_xyz);
        qddd_xyz(end+1,:) = qddd_xyz(end,:);
        subplot(4,1,4)
        plot(tvec, qddd_xyz)
        xlabel('t')
        ylabel('Jerk')
        xline(tpts(2))
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y), transpose(lin_yy_x_z)];
end

