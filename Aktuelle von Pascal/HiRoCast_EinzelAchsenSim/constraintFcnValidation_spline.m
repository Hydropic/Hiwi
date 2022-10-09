function [c,ceq] = constraintFcnValidation_spline(optimization_values, splineDiscretization, axesPointConfigs, max_values, min_values, jerkBoundaries)
    ceq =[];
    c=[];
    %% =========Feste Variablen==========================================
    achsstellungen = axesPointConfigs.'

    wayPoints = [];

    for p = 1:height(achsstellungen)
        [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen(p,:))
        wayPoints(:,p) = tcppunkt(:,1)
    end
    
    for t = 1:width(optimization_values)-1
        if optimization_values(1,t) <= 0
            optimization_values(1,t) = 0
        end
    end

    tvec = 0:0.08:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]
    VelocityBoundaryCondition_y = [0, optimization_values(3,:)]
    VelocityBoundaryCondition_z = [0, optimization_values(4,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(5,:)]
    AccelerationBoundaryCondition_y = [0, optimization_values(6,:)]
    AccelerationBoundaryCondition_z = [0, optimization_values(7,:)]

    % Check, if Time is rising on every Point
    timeRising = false;
    for t = 1:width(optimization_values)-1
        if optimization_values(1,t) >= optimization_values(1,t+1)
            timeRising = true
            break
        else
            timeRising = false
        end
    end

    if timeRising == false
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

    qddd_xyz = diff(qdd_xyz);

    % Ruck Differenz an Kollisionspunkt    
    diffValue(1:length(qddd_xyz), 1:width(qddd_xyz)) = 0;
    for a = 1:width(qddd_xyz)
        diffValue(1, a) = 0
        for h = 2:length(qddd_xyz)
            diffValue(h,a) = qddd_xyz(h,a) - qddd_xyz(h-1,a)
        end
    end

    for timePoint = 1:width(optimization_values)-1
        % Nur den Ruck in der Nähe des Kollisionspunktes aufnehmen
        timePoint = optimization_values(1, timePoint)
        diffValuePoint = [];
    
        % Determine all values around the time
        diffValues_pos = tvec - timePoint
        diffValues_neg = tvec - timePoint
        diffValues_pos(diffValues_pos > 0) = -inf;
        diffValues_neg(diffValues_neg < 0) = inf;
        [~, indexOfMax_pos] = max(diffValues_pos)
        [~, indexOfMax_neg] = min(diffValues_neg)
        maxValue_pos = tvec(indexOfMax_pos)
        maxValue_neg = tvec(indexOfMax_neg)
    
        diffValuePoint(1,:) = qddd_xyz(indexOfMax_pos-2,:);
        diffValuePoint(2,:) = qddd_xyz(indexOfMax_pos-1,:);
        diffValuePoint(3,:) = qddd_xyz(indexOfMax_pos,:);
        diffValuePoint(4,:) = qddd_xyz(indexOfMax_neg,:);
        diffValuePoint(5,:) = qddd_xyz(indexOfMax_neg+1,:);
        diffValuePoint(6,:) = qddd_xyz(indexOfMax_neg+2,:);
    
        max_spline_ddd_x = max(diffValuePoint(:,1))
        max_spline_ddd_y = max(diffValuePoint(:,2))
        max_spline_ddd_z = max(diffValuePoint(:,3))
    
        min_spline_ddd_x = min(diffValuePoint(:,1))
        min_spline_ddd_y = min(diffValuePoint(:,2))
        min_spline_ddd_z = min(diffValuePoint(:,3))
    
            c(end+1) = max_spline_ddd_x - jerkBoundaries
            c(end+1) = max_spline_ddd_y - jerkBoundaries
            c(end+1) = max_spline_ddd_z - jerkBoundaries
    
            c(end+1) = -jerkBoundaries - min_spline_ddd_x
            c(end+1) = -jerkBoundaries - min_spline_ddd_y
            c(end+1) = -jerkBoundaries - min_spline_ddd_z
    end



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

        c(end+1) = max_spline_d_x - max_values(2,1)
        c(end+1) = max_spline_d_y - max_values(2,2)
        c(end+1) = max_spline_d_z - max_values(2,3)
    
        c(end+1) = min_values(2,1) - min_spline_d_x 
        c(end+1) = min_values(2,2) - min_spline_d_y
        c(end+1) = min_values(2,3) - min_spline_d_z
    
        c(end+1) = max_spline_dd_x - max_values(3,1)
        c(end+1) = max_spline_dd_y - max_values(3,2)
        c(end+1) = max_spline_dd_z - max_values(3,3)
    
        c(end+1) = min_values(3,1) - min_spline_dd_x
        c(end+1) = min_values(3,2) - min_spline_dd_y
        c(end+1) = min_values(3,3) - min_spline_dd_z
    else
        for w = 1:(3*6)
            c(end+1) = 10; 
        end
    end

%% =========Ausgabe auf der Konsole=============================     
    c_string = join(string( find(c > 0)   ), ',');   
    position_c_fail = find(c > 0);
    c_value_string = [];
    %c_value_string = join(string( find(c > 0)   ), ',');
    for z = 1: length(position_c_fail)
        c_value_string(end+1,1) = c(position_c_fail(z));
    end    
    %ausgabe der anzah der c Bedingungen und die anzahl der verletzten
    if ismissing(c_string) == false
        fprintf('c(%d, %d) [%s] \n',length(c),length(find(c-1e-6 > 0)) ,strjoin(string(round(c_value_string,1)),', '));
    else
        fprintf('Keine Bedingung verletzt')
        fprintf(num2str(length(c)));       
    end
   
end