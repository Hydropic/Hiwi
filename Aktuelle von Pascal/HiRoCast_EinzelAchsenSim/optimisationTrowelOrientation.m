function [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations, setToNullOrientation, startConfig, goalConfig)

    global init_optimization_values;
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;   

    %% ========Umorientierung auf Basis von TCP KOS====================
    [path_angular_deflection_old] = determinationAccelerationTCP(optimized_translational_values);

    [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection_old,optimized_translational_values);
    % optimized_translational_values_oriented --> gibt an, wie die Achse 6
    % orentiert werden müsste um die Ausgleichsbewegungn zu realisieren

    init_optimization_values = optimized_translational_values_oriented(:,7);

    % global sumAngularDifference;
% 
% varianceStartUEndOrientation = 0.05; % rad
% 
%     acceptableOrientationDifference = 0.01 %°
%     numOfIterations = 30;
%     middleOneConfigUse = true;
%     middleTwoConfigUse = false;
%     xOrientationNull = false;
%     yOrientationNull = true;
%     numSamples = height(optimized_translational_values);
%     for t = 1:numSamples-1
%         timesteps(t, 1) = optimized_translational_values(t,1)
%     end
    % Angle Defection ermitteln
    % 0 Stellung der 6 Achse drauf rechnen
    % Initiale Werte setzen, indem eine Ausgleichskurve erzeugt wird
    % Grenzen für die Achse 6 und Zeitintervalle dazwischen setzen
    
    %% ==================== lb u. ub ================================    
    % Definig ub lb and options
    max_jointangle = deg2rad([350]);
    min_jointangle = deg2rad([-350]);

    max_values(1:20) = max_jointangle;
    min_values(1:20) = min_jointangle;

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
        'nonlcon', @(optimization_values)constraintFcnValidation_A6(optimization_values, optimized_translational_values_oriented), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    [x,fval,eflag,output] = fmincon(problem);
    figure
    plot(optimized_translational_values_oriented(:,7));
    optimized_translational_values(:,7) = x;

    show_spline(optimized_translational_values, 'Achse 6');
            
    function objective = optimization_task(optimization_values)
        init_optimization_values
        angleDiff = 0
        for a = 1:length(optimization_values)
            angleDiff = angleDiff + abs(optimization_values(a) - init_optimization_values(a))
        end
    objective = angleDiff;
    end
end
