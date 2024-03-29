function [x,fval,eflag,output] = optimisationTrowelOrientation()
   
close all;
clearvars -except setToNullOrientation;
global sumAngularDifference;

varianceStartUEndOrientation = 0.05; % rad
    passes = 50;
    numOfIterations = 10;
   
    % Laden der Zeitoptimiertenoptimierten Bahn füe Achse 1, 2 und 3
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;
    
    numSamples = height(optimized_translational_values);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% timesteps
    for t = 1:numSamples-1
        timesteps(t, 1) = optimized_translational_values(t,1)
    end

    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, 1);

   
    discretizedsSplinePathPlot(:,1) = position_TCP(:,1)
    discretizedsSplinePathPlot(:,2) = position_TCP(:,2)
    discretizedsSplinePathPlot(:,3) = position_TCP(:,3)
    discretizedsSplinePathPlot(:,4) = position_TCP(:,4)
    discretizedsSplinePathPlot(:,5) = position_TCP(:,2)
    discretizedsSplinePathPlot(:,6) = position_TCP(:,3)
    discretizedsSplinePathPlot(:,7) = position_TCP(:,4)

    show_spline(discretizedsSplinePathPlot, 'y (um Achse 6), x (um Achse 5)');


    %% ========Umorientierung auf Basis von TCP KOS====================
    [path_angular_deflection_old] = determinationAccelerationTCP(optimized_translational_values);

    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, 1);
    [row_initial_values, collum_initial_values] = size(optimized_translational_values);

    
    %% ==================== lb u. ub ================================
    
    % Definig ub lb and options
    max_jointangle = deg2rad([350,120,350]);
    min_jointangle = deg2rad([-350,-120,-350]);
    min_values = repmat(min_jointangle,[size(optimization_values,1),1]); 
    max_values = repmat(max_jointangle,[size(optimization_values,1),1]);
    [eins, zwei] = size(max_values);
    
    % Fix Start and End Position
    for s = 1:zwei
        if s == 1
            min_values(1, s) = optimization_values(1, s) - varianceStartUEndOrientation;
            max_values(1, s) = optimization_values(1, s) + varianceStartUEndOrientation;
            min_values(end, s) = optimization_values(end, s) - varianceStartUEndOrientation;
            max_values(end, s) = optimization_values(end, s) + varianceStartUEndOrientation;
        else
            min_values(1, s) = optimization_values(1, s);
            max_values(1, s) = optimization_values(1, s);
            min_values(end, s) = optimization_values(end, s);
            max_values(end, s) = optimization_values(end, s);
        end
    end
    
    % Fix middleOne Position
    if middleOneConfigUse
        for i = 1:zwei-1
            min_values(middleOneConfigPosition, i) = middleOneConfig(i) - 0.1;
            max_values(middleOneConfigPosition, i) = middleOneConfig(i) + 0.1;
        end
    end
    
    % Fix middleOne Position
    if middleTwoConfigUse
        for i = 1:zwei-1
            min_values(middleTwoConfigPosition, i) = middleTwoConfig(i) - 0.1;
            max_values(middleTwoConfigPosition, i) = middleTwoConfig(i) + 0.1;
        end
    end

    % plot(path_angular_deflection_old)
    
    
    [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection_old,optimized_translational_values, xOrientationNull, yOrientationNull);

    optimized_translational_values(:,6) = optimized_translational_values_oriented(:,6);
    optimized_translational_values(:,7) = optimized_translational_values_oriented(:,7);
        
    % Setzen der initialen Werte
    init_optimization_values(:,1) = optimized_translational_values_oriented(:,5);
    init_optimization_values(:,2) = optimized_translational_values_oriented(:,6);
    init_optimization_values(:,3) = optimized_translational_values_oriented(:,7);
    optimization_values(:,1) = init_optimization_values(:,1);
    optimization_values(:,2) = init_optimization_values(:,2);
    optimization_values(:,3) = init_optimization_values(:,3);
    
    for a = 1:length(optimized_translational_values_oriented)
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(example.x(a,2:7));
        eulerZYX_save(a,:) = eulerZYX(1,:);
    end
    figure;
    plot(init_optimization_values)
    %% =========Optimierung Achswinkelstellungen===========================
    % Optimierung der 5 und 6 Achse mit den Pendelmodellergebnissen
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",300000,...
        "MaxIterations",numOfIterations, ...
        "StepTolerance",1e-10, ...
        "OptimalityTolerance",1e-10, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.0001, ...
        "DiffMaxChange", 10, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize"], ...
        "Display",'iter','ConstraintTolerance',0.001);
       
    problem = createOptimProblem('fmincon',...
        'x0',init_optimization_values, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation_formXY(optimization_values,init_optimization_values,optimized_translational_values,path_angular_deflection, passes, acceptableOrientationDifference), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    [x,fval,eflag,output] = fmincon(problem);
    
      
            
    function objective = optimization_task(optimization_values)
        if isempty(sumAngularDifference)
            example2 = matfile('sum_Angular_Difference.mat');
            objective = example2.sumAngularDifference; 
        else
            objective = sumAngularDifference;
        end                
%             timeintervals = optimization_values(1:size(optimization_values,1)-1,1)
%             base_points = optimization_values(:,2:size(optimization_values,2));
%             objective = sum(timeintervals); 
%             for i=1:size(base_points,2)
%                 [achsstellung,vilocity,acceleration,jerk,time,splinePunkt] = spline(base_points(:,i),timeintervals,false);
%                 objective = objective + 0.1*trapz((acceleration.^2)); %TODO 
%             end
    end
end
