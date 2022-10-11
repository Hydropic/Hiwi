function [Position_xyz, timeLine] = generateTCPPath(optimization_values, wayPoints, splineDiscretization, visualizeTCPPath, min_values, max_values, jerkBoundaries)
[breite, hoehe] = size(optimization_values)

if breite == 7
       for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]
    VelocityBoundaryCondition_y = [0, optimization_values(3,:)]
    VelocityBoundaryCondition_z = [0, optimization_values(4,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(5,:)]
    AccelerationBoundaryCondition_y = [0, optimization_values(6,:)]
    AccelerationBoundaryCondition_z = [0, optimization_values(7,:)]

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

    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y), transpose(lin_yy_x_z)];

elseif breite == 5
       for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]
    VelocityBoundaryCondition_y = [0, optimization_values(3,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(4,:)]
    AccelerationBoundaryCondition_y = [0, optimization_values(5,:)]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)
    [q_y,qd_y,qdd_y,pp_y] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y)

    q_xyz = [transpose(q_x), transpose(q_y)];
    qd_xyz = [transpose(qd_x), transpose(qd_y)];
    qdd_xyz = [transpose(qdd_x), transpose(qdd_y)];
    pp_xyz = [transpose(pp_x), transpose(pp_y)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);

    lin_xx_x_y = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), lin_xx_x_y);

    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y)];
elseif breite == 3
       for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(3,:)]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)

    q_xyz = [transpose(q_x)];
    qd_xyz = [transpose(qd_x)];
    qdd_xyz = [transpose(qdd_x)];
    pp_xyz = [transpose(pp_x)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);


    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
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
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x)];
end
 
end

