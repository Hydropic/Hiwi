function [c,ceq] = constraintFcnValidation_spline(optimization_values, splineDiscretization, startConfig, middleOneConfig, goalConfig)
ceq =[];
c=[];
%% =========Feste Variablen==========================================

    achsstellungen(1,:) = startConfig
    achsstellungen(2,:) = middleOneConfig
    achsstellungen(3,:) = goalConfig

    wayPoints = [];

    for p = 1:height(achsstellungen)
        [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen(p,:))
        wayPoints(:,p) = tcppunkt(:,1)
    end

    
    tvec = 0:0.01:optimization_values(1, 3);
    tpts = optimization_values(1, 1:3);
    % tpts = [0,1.2,2.3];

    VelocityBoundaryCondition_x = [0 optimization_values(2, 1) 0]
    VelocityBoundaryCondition_y = [0 optimization_values(2, 2) 0]
    VelocityBoundaryCondition_z = [0 optimization_values(2, 3) 0]

    AccelerationBoundaryCondition_x = [0 optimization_values(3, 1) 0]
    AccelerationBoundaryCondition_y = [0 optimization_values(3, 2) 0]
    AccelerationBoundaryCondition_z = [0 optimization_values(3, 3) 0]

    numSamples = 100;
    % [q,qd,qdd,tvec,pp] = trapveltraj(wayPoints(1,:),numSamples,"EndTime",5)
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

    max_values = [2 3]
    min_values = [-2 -3]   
      
        c(end+1) = max_spline_d_x - max_values(1)
        c(end+1) = max_spline_d_y - max_values(1)
        c(end+1) = max_spline_d_z - max_values(1)
    
        c(end+1) = min_values(1) - min_spline_d_x 
        c(end+1) = min_values(1) - min_spline_d_y
        c(end+1) = min_values(1) - min_spline_d_z
    
        c(end+1) = max_spline_dd_x - max_values(2)
        c(end+1) = max_spline_dd_y - max_values(2)
        c(end+1) = max_spline_dd_z - max_values(2)
    
        c(end+1) = min_values(2) - min_spline_dd_x
        c(end+1) = min_values(2) - min_spline_dd_y
        c(end+1) = min_values(2) - min_spline_dd_z

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