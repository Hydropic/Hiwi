function [] = optimisationTrowelOrientation()
    % Laden der Zeitoptimiertenoptimierten Bahn füe Achse 1, 2 und 3
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;
    [row_initial_values, collum_initial_values] = size(optimized_translational_values);

    % split valuse
    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values);

        % Definig ub lb and options
    max_jointangle = deg2rad([120,350]);
    min_jointangle = deg2rad([-120,-350]);
    min_values = repmat(min_jointangle,[size(optimization_values,1),1]); 
    max_values = repmat(max_jointangle,[size(optimization_values,1),1]);
    
    [eins, zwei] = size(max_values);
    % Fix Start and End Position
    for s = 1:zwei
        min_values(1, s) = optimization_values(1, s);
        max_values(1, s) = optimization_values(1, s);
        min_values(end, s) = optimization_values(end, s);
        max_values(end, s) = optimization_values(end, s);
    end

    %% =========Winkelausschlag aus Pendelmodell=============================== 
    % Pendelsimulation aufrufen
    [path_angular_deflection] = pendelSimulation(position_TCP);
    [optimized_translational_values_oriented] = matchPendelPathData(path_angular_deflection,optimized_translational_values_clear);
    x = optimized_translational_values_oriented;
    save('SimResults.mat','x');

%     figure
%     patch(path_angular_deflection(:,1),path_angular_deflection(:,2),path_angular_deflection(:,3),path_angular_deflection(:,3),'EdgeColor','interp','Marker','o','MarkerFaceColor','flat');
%     view(3)
%     colorbar;
       
    % Update all valuse
    [optimized_translational_values_update] = updateTimeAndAxes(optimization_values, optimized_translational_values)

    % split valuse
    [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values);

    %% =========Optimierung Achswinkelstellungen===========================
    % Optimierung der 5 und 6 Achse mit den Pendelmodellergebnissen
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",1,...
        "MaxIterations",1, ...
        "StepTolerance",1e-4, ...
        "OptimalityTolerance",1e-54, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.01, ...
        "DiffMaxChange", 10, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
        "Display",'iter','ConstraintTolerance',0.5);
    

    
    problem = createOptimProblem('fmincon',...
        'x0',optimization_values, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation_pendel(optimized_translational_values_update,path_angular_deflection,optimization_values), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    [x,fval,eflag,output] = fmincon(problem);
      
    %Plotting output
%     KSetUp;
%     KPfadgenerator;
%     showBahn = x;
%     showBahn(:,1) = [];
%     laufbahn(robot,showBahn,1,true)
%    show_spline(x, 'Bad Output');
    
    % Update all valuse
    [optimized_translational_values_update] = updateTimeAndAxes(x, optimized_translational_values);
    



    function objective = optimization_task(optimization_values)
        angleDifference = [];

        % Umrechnung der Achsstellungen in TCP Koordinaten
        example = matfile('SimResults.mat');
        optimized_translational_values = example.x;  

        example2 = matfile('path_angular_deflection.mat');
        path_angular_deflection = example2.path_angular_deflection;

        % Update all valuse
        [optimized_translational_values_update] = updateTimeAndAxes(optimization_values, optimized_translational_values)
        
        % split valuse
        [position_TCP, angleTCP, optimization_values] = prepairValuse_5_6(optimized_translational_values_update);

        [row_initial_values2, collum_initial_values2] = size(optimized_translational_values_update);      
        for u = 1:row_initial_values2
            angleDifference(end+1) = abs(path_angular_deflection(u, 2) - position_TCP(u, 2));
            angleDifference(end+1) = abs(path_angular_deflection(u, 3) - position_TCP(u, 3));
            %angleDifference(end+1) = abs(path_angular_deflection(u, 4) - optimized_translational_values_TCP_orientation(u, 4));
        end
        objective = sum(angleDifference); 
 end
end

