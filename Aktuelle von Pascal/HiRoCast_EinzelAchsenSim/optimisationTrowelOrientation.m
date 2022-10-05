function [x,fval,eflag,output] = optimisationTrowelOrientation(setToNullOrientation, startConfig, goalConfig)
   
close all;
clearvars -except setToNullOrientation startConfig goalConfig sumAngularDifference;
global sumAngularDifference;

varianceStartUEndOrientation = 0.05; % rad
    passes = 50;
    numOfIterations = 10;
    if setToNullOrientation
        acceptableOrientationDifference = 0.01 %°
        numOfIterations = 30;
        middleOneConfigUse = true;
        middleTwoConfigUse = false;
        xOrientationNull = false;
        yOrientationNull = true;
    else
        acceptableOrientationDifference = 0.5 %°
        numOfIterations = 20;
        middleOneConfigUse = false;
        middleTwoConfigUse = false;
        xOrientationNull = false;
        yOrientationNull = false;
    end    
   
    % Laden der Zeitoptimiertenoptimierten Bahn füe Achse 1, 2 und 3
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;
    
    % =======Anfangs- und Endorientierung händisch auf 0 setzen==========
    
    % % TODO: AUTOMATISCH ERMITTELN
    startConfig(:, 1) = []
    startConfig(:, 1) = []
    startConfig(:, 1) = []
    goalConfig(:, 1) = []
    goalConfig(:, 1) = []
    goalConfig(:, 1) = []
    
    optimized_translational_values(1,5) = startConfig(1,1);
    optimized_translational_values(1,6) = startConfig(1,2);
    optimized_translational_values(1,7) = startConfig(1,3);
    optimized_translational_values(end,5) = goalConfig(1,1);
    optimized_translational_values(end,6) = goalConfig(1,2);
    optimized_translational_values(end,7) = goalConfig(1,3);
    
    middleOneConfig  = deg2rad([-203.552425, 57.791275, 193.079891]);
    middleOneConfigPosition = 11;
    
    middleTwoConfig  = deg2rad([157.351607, 53.478628, 133.945374]);
    middleTwoConfigPosition = 15;
    
    numSamples = height(optimized_translational_values);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% timesteps
    for t = 1:numSamples-1
        timesteps(t, 1) = optimized_translational_values(t,1)
    end
    
    if middleOneConfigUse || middleTwoConfigUse
        minJerkPath = pfadGeneratorSimple(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition-1, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition-1);
        
        optimized_translational_values(:,5) = minJerkPath(:,1);
        optimized_translational_values(:,6) = minJerkPath(:,2);
        optimized_translational_values(:,7) = minJerkPath(:,3);
    end
    show_spline(optimized_translational_values,'hä')
    %% ========Umorientierung auf Basis von TCP KOS====================
    [path_angular_deflection_old] = determinationAccelerationTCP(optimized_translational_values);

    % plot(path_angular_deflection_old)
    
    % %% =========Winkelausschlag aus Pendelmodell=============================== 
    % [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, 1);
    % 
    % % Pendelsimulation aufrufen
    % [path_angular_deflection_old] = pendelSimulation(position_TCP); % path_angular_deflection = [Zeit, rotation um y, Rotation um x]
     
    % figure
    % plot(path_angular_deflection)
    
    % EndTimeRoboPath = ZeitintervallDerPendelbewegung;
    % countInterval = round((EndTimeRoboPath-sum(optimized_translational_values(:,1)))/timeIntevalForbalance);
    % if countInterval > 0
    %     for t = 1:countInterval
    %         optimized_translational_values(end+1,:) = [timeIntevalForbalance, optimized_translational_values(end,2), optimized_translational_values(end,3), optimized_translational_values(end,4), optimized_translational_values(end,5), optimized_translational_values(end,6), optimized_translational_values(end,7)];
    %     end
    % end

    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, 1);
    [row_initial_values, collum_initial_values] = size(optimized_translational_values);
    
    
    % % % % % % % % % % plot(optimized_translational_values);
    % % % % % % % % % % plot(optimized_translational_values_clear)
    
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
