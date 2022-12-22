function [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations, setToNullOrientation)

    %% ========Umorientierung auf Basis von TCP KOS====================
    [path_angular_deflection_old] = determinationAccelerationTCP(optimized_translational_values);

    %ZRot in zeile 8 integrieren + addition der Rotationen
    [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection_old,optimized_translational_values);

    %% =========Optimierung Achswinkelstellungen===========================
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",300000,...
        "MaxIterations",numOfIterations, ...
        "StepTolerance",1e-18, ...
        "OptimalityTolerance",1e-18, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.001, ...
        "DiffMaxChange", 10, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize"], ...
        "Display",'iter','ConstraintTolerance',0.001);
       
    problem = createOptimProblem('fmincon',...
        'x0',init_optimization_values, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation_orientationY(optimization_values, optimized_translational_values_oriented), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
%     [x,fval,eflag,output] = fmincon(problem);
% 
%         example = matfile('SimResults.mat');
%     optimized_translational_values = example.x;   
% 
%     figure
%     plot(optimized_translational_values_oriented(:,7));
%     optimized_translational_values(:,7) = x;
%     hold on;
%     plot(optimized_translational_values(:,7));
% 
%     show_spline(optimized_translational_values, 'Alle Achsen');
% 
%     for z = 1:length(optimized_translational_values)
%         [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(optimized_translational_values(z,2:7));
%         optimized_translational_values(z,7) = eulerZYX(2);
%     end
%     show_spline(optimized_translational_values, 'y (um Achse 6), x (um Achse 5)');
% 
%     show_spline(optimized_translational_values, 'Achse 6');
%            
    function objective = optimization_task(optimization_values)
        init_optimization_values
        angleDiff = 0
        for a = 1:length(optimization_values)
            angleDiff = angleDiff + abs(optimization_values(a) - init_optimization_values(a))
        end
    objective = angleDiff;
    end
end
