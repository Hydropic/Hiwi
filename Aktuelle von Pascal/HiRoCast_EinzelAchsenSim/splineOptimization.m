function [x, optiResuls] = splineOptimization(maxIterations, splineDiscretization, startConfig, middleOneConfig, goalConfig)

    % set initial Valuse

    tpts = [0,0.9 ,1.0];
    % tpts = [0,1.2,2.3];

    VelocityBoundaryCondition_x = [0 1 0]
    VelocityBoundaryCondition_y = [0 -1 0]
    VelocityBoundaryCondition_z = [0 0 0]

    VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x(2), VelocityBoundaryCondition_y(2), VelocityBoundaryCondition_z(2)]

    AccelerationBoundaryCondition_x = [0 -3 0]
    AccelerationBoundaryCondition_y = [0 2 0]
    AccelerationBoundaryCondition_z = [0 -0.3 0]

    AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x(2), AccelerationBoundaryCondition_y(2), AccelerationBoundaryCondition_z(2)]


    init_ax_values = [tpts; VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];
   
    % Defineig options für das Optimalsteuerungsprob.
    opts = optimoptions(@fmincon, ...
        'Algorithm','interior-point', ...
        "MaxFunctionEvaluations",50000,...
        "MaxIterations",maxIterations, ...
        "StepTolerance",1e-17, ...
        "OptimalityTolerance",1e-17, ...
        "EnableFeasibilityMode",true, ...
        "DiffMinChange", 0.01, ... 
        "DiffMaxChange", 2, ...
        "SubproblemAlgorithm",'factorization', ...
        "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
        "Display",'iter','ConstraintTolerance',0.00018);

    min_values = [0 0.1 1.0; 
                  -2.2 -2.2 -2.2; 
                  -4.0 -3.0 -8.0]

    max_values = [0 2.3 3.5; 
                  2.2 2.2 2.2; 
                  4.0 3.0 8.0]
    
        
    optimization_values = init_ax_values;
    problem = createOptimProblem('fmincon',...
        'x0',init_ax_values, ...
        'objective',@optimization_task,...
        'nonlcon', @(optimization_values)constraintFcnValidation_spline(optimization_values, splineDiscretization, startConfig, middleOneConfig, goalConfig), ...
        'lb',min_values,...
        'ub',max_values, ...
        'options',opts);
        
    [x,fval,eflag,output] = fmincon(problem);
    
    optiResuls = [fval, output.constrviolation]

    end


    function objective = optimization_task(optimization_values)
        objective = optimization_values(1, 3)
    end