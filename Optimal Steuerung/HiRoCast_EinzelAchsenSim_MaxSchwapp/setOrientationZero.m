function [optimized_translational_values] = setOrientationZero(optimized_translational_values)
    % ==== Initiale Werte festlegen ======
    startConfig = deg2rad([154.448541, -68.120258, 125.846874, -149.924251, 61.343497, 164.479036]);
    goalConfig  = deg2rad([82.978790, -43.681070, 83.414503, -172.796796, 39.956763, 174.466527]);
    
    optimized_translational_values(1,5) = startConfig(1,1);
    optimized_translational_values(1,6) = startConfig(1,2);
    optimized_translational_values(1,7) = startConfig(1,3);
    optimized_translational_values(end,5) = goalConfig(1,1);
    optimized_translational_values(end,6) = goalConfig(1,2);
    optimized_translational_values(end,7) = goalConfig(1,3);
    
    middleOneConfigUse = true;
    middleOneConfig  = deg2rad([98.240000, -64.879820, 119.410547, -204.219076, 56.985239, 0]);
    middleOneConfigPosition = 10;
    
    middleTwoConfigUse = false;
    middleTwoConfig  = deg2rad([20.02, -60.61, 180.26, 71.82, -81.59, -24.3]);
    middleTwoConfigPosition = 15;
    
    numSamples = height(optimized_translational_values);
    
    minJerkPath = pfadGenerator(startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition);
    
    %% ================ set ini Valuse ===============
    init_optimization_values(:,1) = minJerkPath(:,4);
    init_optimization_values(:,2) = minJerkPath(:,5);
    init_optimization_values(:,3) = minJerkPath(:,6);
    optimization_values(:,1) = minJerkPath(:,4);
    optimization_values(:,2) = minJerkPath(:,5);
    optimization_values(:,3) = minJerkPath(:,6);

    % Set Orientation to Zero
    for k = 1:height(optimized_translational_values)
        if k == 1
            path_angular_deflection(k,1) = optimized_translational_values(k,1);
            path_angular_deflection(k,2) = 0;
            path_angular_deflection(k,3) = 0;
        else
            path_angular_deflection(k,1) = path_angular_deflection(k-1,1) + optimized_translational_values(k,1);
            path_angular_deflection(k,2) = 0;
            path_angular_deflection(k,3) = 0;
        end
    end
    
    % input der achsstellungen
    init_ax_values = [ones(size(minJerkPath,1),1) minJerkPath];
    init_ax_values(:,1) = optimized_translational_values(:,1); %Anfängliche Beschleunigung = 0,3m/s???
    
    % Defineig options für das Optimalsteuerungsprob.
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",50000,...
        "MaxIterations",100, ...
        "StepTolerance",1e-15, ...
        "OptimalityTolerance",1e-15, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.01, ...
        "DiffMaxChange", 1, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
        "Display",'iter','ConstraintTolerance',0.05);
    
    % Definig ub lb and options
    max_jointangle = deg2rad([185,14,144,350,120,350]);
    min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
    min_values = optimized_translational_values; 
    max_values = optimized_translational_values;
    
    [eins, zwei] = size(max_values);
    % Fix Start and End Position
    for i = 1:zwei-1
        min_values(1, i+1) = minJerkPath(1, i);
        max_values(1, i+1) = minJerkPath(1, i);
        min_values(end, i+1) = minJerkPath(end, i);
        max_values(end, i+1) = minJerkPath(end, i);
    end
    
    % Fix middleOne Position
    if middleOneConfigUse
        for i = 1:zwei-1
            middleOneConfig
            min_values(middleOneConfigPosition, i+1) = middleOneConfig(i) - 0.05;
            max_values(middleOneConfigPosition, i+1) = middleOneConfig(i) + 0.05;
        end
    end
    
    % Fix middleTwo Position
    if middleTwoConfigUse
        for i = 1:zwei-1
            min_values(middleTwoConfigPosition, i+1) = minJerkPath(middleTwoConfig(i), i);
            max_values(middleTwoConfigPosition, i+1) = minJerkPath(middleTwoConfig(i), i);
        end
    end
    
    % Fix Achs 4, 5 und 6
    for i = 1:eins
        min_values(i, 2) = minJerkPath(1, 1);
        max_values(i, 2) = minJerkPath(1, 1);
        min_values(i, 3) = minJerkPath(1, 2);
        max_values(i, 3) = minJerkPath(1, 2);
        min_values(i, 4) = minJerkPath(1, 3);
        max_values(i, 4) = minJerkPath(1, 3);
    end
    

    problem = createOptimProblem('fmincon',...
        'x0',init_ax_values, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation_orientation(optimization_values,init_optimization_values,optimized_translational_values,path_angular_deflection), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    [x,fval,eflag,output] = fmincon(problem);
    
    % Save Results
    save('SimResults.mat','x');    
    show_spline(x, 'Bad Output');      
    generierenEmily(x);    
    
    
    function objective = optimization_task(optimization_values)
        example2 = matfile('sum_Orientation_Difference.mat');
        objective2 = example2.sumOrientationDifference; 
        objective = objective2; 
    end
end