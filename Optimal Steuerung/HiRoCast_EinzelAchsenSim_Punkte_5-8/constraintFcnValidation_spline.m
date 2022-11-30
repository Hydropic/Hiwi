function [c,ceq] = constraintFcnValidation_spline(optimalSplineDiscretization, optimization_values_XY, splineDiscretization, axesPointConfigs, max_values, min_values, jerkBoundaries)
    ceq =[];
    c=[];
    %% =========Feste Variablen==========================================
    achsstellungen = axesPointConfigs.'

    wayPoints = [];

    for t = 1:length(optimization_values_XY(1,:))
        if t == 1
            optimization_values_XY(1,t) = optimization_values_XY(1,t)
        else
            optimization_values_XY(1,t) = optimization_values_XY(1,t-1) + optimization_values_XY(1,t)
        end
    end

    for p = 1:height(achsstellungen)
        [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen(p,:))
        wayPoints(:,p) = tcppunkt(:,1)
    end
    
    interval =  round(optimization_values_XY(1, end)/optimalSplineDiscretization, 3)
    tvec = 0:interval:optimization_values_XY(1, end);
    tpts = [0, optimization_values_XY(1,:)]

    VelocityBoundaryCondition_x = [0.07, optimization_values_XY(2,:)]
    VelocityBoundaryCondition_y = [0.8, optimization_values_XY(3,:)]

    AccelerationBoundaryCondition_x = [0.4, optimization_values_XY(4,:)]
    AccelerationBoundaryCondition_y = [0.49, optimization_values_XY(5,:)]

    % Check, if Time is rising on every Point
    timeRising = false;
    for t = 1:width(optimization_values_XY)-1
        if optimization_values_XY(1,t) >= optimization_values_XY(1,t+1)
            timeRising = true
            break
        else
            timeRising = false
        end
    end

    if timeRising == false
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

    qddd_xyz = diff(diff(qdd_xyz));

    % Ruck Differenz an Kollisionspunkt    
    diffValue(1:length(qddd_xyz), 1:width(qddd_xyz)) = 0;
    for a = 1:width(qddd_xyz)
        diffValue(1, a) = 0
        for h = 2:length(qddd_xyz)
            diffValue(h,a) = qddd_xyz(h,a) - qddd_xyz(h-1,a)
        end
    end

    for timePoint = 1:width(optimization_values_XY)-1
        % Nur den Ruck in der Nähe des Kollisionspunktes aufnehmen
        timePoint = optimization_values_XY(1, timePoint)
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
    
        diffValuePoint(3,:) = qddd_xyz(indexOfMax_pos-2,:);
        diffValuePoint(3,:) = qddd_xyz(indexOfMax_pos-1,:);
        diffValuePoint(4,:) = qddd_xyz(indexOfMax_pos,:);
        diffValuePoint(5,:) = qddd_xyz(indexOfMax_pos+1,:);
        diffValuePoint(6,:) = qddd_xyz(indexOfMax_pos+2,:);
    
        max_spline_ddd_x = max(diffValuePoint(:,1))
        max_spline_ddd_y = max(diffValuePoint(:,2))
    
        min_spline_ddd_x = min(diffValuePoint(:,1))
        min_spline_ddd_y = min(diffValuePoint(:,2))
    
            c(end+1) = max_spline_ddd_x - jerkBoundaries
            c(end+1) = max_spline_ddd_y - jerkBoundaries
    
            c(end+1) = -jerkBoundaries - min_spline_ddd_x
            c(end+1) = -jerkBoundaries - min_spline_ddd_y
    end

        % für jeden Bereich seperat

        max_spline_d_x = max(qd_xyz(:,1))
        max_spline_d_y = max(qd_xyz(:,2))
    
        min_spline_d_x = min(qd_xyz(:,1))
        min_spline_d_y = min(qd_xyz(:,2))
    
        max_spline_dd_x = max(qdd_xyz(:,1))
        max_spline_dd_y = max(qdd_xyz(:,2))
    
        min_spline_dd_x = min(qdd_xyz(:,1))
        min_spline_dd_y = min(qdd_xyz(:,2))
    
            c(end+1) = max_spline_d_x - max_values(2,1)
            c(end+1) = max_spline_d_y - max_values(3,1)
        
            c(end+1) = min_values(2,1) - min_spline_d_x 
            c(end+1) = min_values(3,1) - min_spline_d_y
        
            c(end+1) = max_spline_dd_x - max_values(4,1)
            c(end+1) = max_spline_dd_y - max_values(5,1)
        
            c(end+1) = min_values(4,1) - min_spline_dd_x
            c(end+1) = min_values(5,1) - min_spline_dd_y
    	
%     % für jeden Bereich seperat
%     for b = 1:length(max_values)
%         % inderx, wo der die Differenz am geringesten ist
%         if b == 1
%             indexOfStart = 1
%             intervalValuesEnd = optimization_values_XY(1,b) - tvec
%             intervalValuesEnd(intervalValuesEnd < 0) = inf;
%             [~, indexOfEnd] = min(intervalValuesEnd)
%         else
%             intervalValuesStart = optimization_values_XY(1,b-1) - tvec
%             intervalValuesStart(intervalValuesStart < 0) = inf;
%             [~, indexOfStart] = min(intervalValuesStart)
%             indexOfStart = indexOfStart + 1;
% 
%             intervalValuesEnd = optimization_values_XY(1,b) - tvec
%             intervalValuesEnd(intervalValuesEnd < 0) = inf;
%             [~, indexOfEnd] = min(intervalValuesEnd)
%         end
% 
%         max_spline_d_x = max(qd_xyz(indexOfStart:indexOfEnd,1))
%         max_spline_d_y = max(qd_xyz(indexOfStart:indexOfEnd,2))
%     
%         min_spline_d_x = min(qd_xyz(indexOfStart:indexOfEnd,1))
%         min_spline_d_y = min(qd_xyz(indexOfStart:indexOfEnd,2))
%     
%         max_spline_dd_x = max(qdd_xyz(indexOfStart:indexOfEnd,1))
%         max_spline_dd_y = max(qdd_xyz(indexOfStart:indexOfEnd,2))
%     
%         min_spline_dd_x = min(qdd_xyz(indexOfStart:indexOfEnd,1))
%         min_spline_dd_y = min(qdd_xyz(indexOfStart:indexOfEnd,2))
%     
%             c(end+1) = max_spline_d_x - max_values(2,b)
%             c(end+1) = max_spline_d_y - max_values(3,b)
%         
%             c(end+1) = min_values(2,b) - min_spline_d_x 
%             c(end+1) = min_values(3,b) - min_spline_d_y
%         
%             c(end+1) = max_spline_dd_x - max_values(4,b)
%             c(end+1) = max_spline_dd_y - max_values(5,b)
%         
%             c(end+1) = min_values(4,b) - min_spline_dd_x
%             c(end+1) = min_values(5,b) - min_spline_dd_y
%     end
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