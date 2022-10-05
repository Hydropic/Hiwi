function [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations, setToNullOrientation, startConfig, goalConfig)
   
close all;
global sumAngularDifference;

varianceStartUEndOrientation = 0.05; % rad

    acceptableOrientationDifference = 0.01 %°
    numOfIterations = 30;
    middleOneConfigUse = true;
    middleTwoConfigUse = false;
    xOrientationNull = false;
    yOrientationNull = true;
   
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;   
    
    numSamples = height(optimized_translational_values);
    for t = 1:numSamples-1
        timesteps(t, 1) = optimized_translational_values(t,1)
    end

    %% ========Umorientierung auf Basis von TCP KOS====================
    [path_angular_deflection_old] = determinationAccelerationTCP(optimized_translational_values);

    [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection_old,optimized_translational_values);
    

    %% ==================== lb u. ub ================================    
    % Definig ub lb and options
    max_jointangle = deg2rad([350,120,350]);
    min_jointangle = deg2rad([-350,-120,-350]);
    min_values = repmat(min_jointangle,[size(optimization_values,1),1]); 
    max_values = repmat(max_jointangle,[size(optimization_values,1),1]);
    [eins, zwei] = size(max_values); 
    
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
        simRes =  example.x(a,2:7);
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(simRes);
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
        'nonlcon', @(optimization_values)constraintFcnValidation_pendel(optimization_values,init_optimization_values,optimized_translational_values,path_angular_deflection, passes, acceptableOrientationDifference), ...
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