function [optimized_transAndRot_values] = optimisationTrowelOrientation(optimized_translational_values)
    
    %% =========Winkelausschlag aus Pendelmodell===============================
    % Umrechnung der Achsstellungen in TCP Koordinaten
    [row_initial_values, collum_initial_values] = size(optimized_translational_values);
    
    optimized_translational_values_TCP = [];
    
    for i= 1:row_initial_values
        p = [optimized_translational_values(i, 2), optimized_translational_values(i, 3), optimized_translational_values(i, 4), optimized_translational_values(i, 5), optimized_translational_values(i, 6), optimized_translational_values(i, 7)];
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p);
    
        optimized_translational_values_TCP(end+1,1) = optimized_translational_values(i, 1);
        optimized_translational_values_TCP(end,2) = pos(1);
        optimized_translational_values_TCP(end,3) = pos(2);
        optimized_translational_values_TCP(end,4) = pos(3);
    end
    
    % Pendelsimulation aufrufen
    [path_angular_deflection] = pendelSimulation(optimized_translational_values_TCP);
    
    
    %% =========Optimierung Achswinkelstellungen===============================
    % Optimierung der 5 und 6 Achse mit den Pendelmodellergebnissen
    % Defineig options für das Optimalsteuerungsprob.
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",1000,...
        "MaxIterations",10000, ...
        "StepTolerance",1e-5, ...
        "OptimalityTolerance",1e-5, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.1, ...
        "DiffMaxChange", 10, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
        "Display",'iter','ConstraintTolerance',0.05);
    
    % Definig ub lb and options
    max_jointangle = deg2rad([185,14,144,350,120,350]);
    min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
    min_values = repmat(cat(2,[0],min_jointangle),[size(optimized_translational_values,1),1]); 
    max_values = repmat(cat(2,[0.2],max_jointangle),[size(optimized_translational_values,1),1]);
    
    [eins, zwei] = size(max_values);
    % Fix Start and End Position
    for i = 1:zwei-1
        min_values(1, i+1) = minJerkPath(1, i);
        max_values(1, i+1) = minJerkPath(1, i);
        min_values(end, i+1) = minJerkPath(end, i);
        max_values(end, i+1) = minJerkPath(end, i);
    end
    
    % Fix der Zeitintervalle und Achsen 1, 2, 3, 4
    for i = 1:eins
        min_values(i, 1) = optimized_translational_values(1, 1);
        max_values(i, 1) = optimized_translational_values(1, 1);
        min_values(i, 2) = optimized_translational_values(1, 2);
        max_values(i, 2) = optimized_translational_values(1, 2);
        min_values(i, 3) = optimized_translational_values(1, 3);
        max_values(i, 3) = optimized_translational_values(1, 3);
        min_values(i, 4) = optimized_translational_values(1, 4);
        max_values(i, 5) = optimized_translational_values(1, 4);
    end
    
    %% Für Testzwecke
    % [eins, zwei] = size(init_ax_values);
    % for i = 1:eins-1
    %     for a = 1:zwei
    %         init_ax_values(i,a) = 0.1;
    %     end
    % end
    optimized_translational_values_minusTime = optimized_translational_values;
    optimized_translational_values_minusTime(1,:) = []
    callFrom = "Pendel";
    problem = createOptimProblem('fmincon',...
        'x0',optimized_translational_values_minusTime, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,"pendel"), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    %[x,fval,eflag,output] = fmincon(problem);
    %gs = GlobalSearch( 'FunctionTolerance' ,2e-4, 'NumTrialPoints' ,2000, 'UseParallel' ,true)
    ms = MultiStart('UseParallel',true);
    [x,f] = run(ms,problem,1);
    
    optimized_transAndRot_values = x;
    
    %Plotting output
    % KSetUp;
    % showBahn = x;
    % showBahn(:,1) = [];
    % laufbahn(robot,showBahn,1,true)
    %show_spline(x, 'Bad Output');
    
    
    function objective = optimization_task(optimization_values)
    % Umrechnung der Achsstellungen in TCP Koordinaten
    angleDifference = [];
    optimized_translational_values_TCP = [];
    [row_initial_values, collum_initial_values] = size(optimization_values);
     

    for i= 1:row_initial_values
        p = [optimization_values(i, 2), optimization_values(i, 3), optimization_values(i, 4), optimization_values(i, 5), optimization_values(i, 6), optimization_values(i, 7)];
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p);
    
        optimized_translational_values_TCP_orientation(end+1,1) = optimization_values(i, 1);
        optimized_translational_values_TCP_orientation(end,2) = eulerXYZ(1);
        optimized_translational_values_TCP_orientation(end,3) = eulerXYZ(2);
        optimized_translational_values_TCP_orientation(end,4) = eulerXYZ(3);
    end    
    for i = 1:length(optimized_translational_values_TCP)
        angleDifference(end+1) = orientationsPendulum(i, 2) - ptimized_translational_values_TCP_orientation(i, 2);
        angleDifference(end+1) = orientationsPendulum(i, 3) - ptimized_translational_values_TCP_orientation(i, 3);
        angleDifference(end+1) = orientationsPendulum(i, 4) - ptimized_translational_values_TCP_orientation(i, 4);
    end
    objective = sum(angleDifference); 
    for i=1:size(base_points,2)
        %[achsstellung,vilocity,acceleration,yerk,time,splinePunkt] = spline(base_points(:,i),timeintervals,false);
        %0.0001
        objective = objective;
        %+ 0.00001 * trapz((vilocity.^2)); %TODO 
    end
end
end

