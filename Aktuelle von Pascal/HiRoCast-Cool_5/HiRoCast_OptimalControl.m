close all
%Variablen
regression = load("regressionKat2.mat");
simulation_data = readtable('simulationData.txt');

%Load data
% load('optimized_input.mat')
% init_values = output.bestfeasible.x;
% init_values(:,1) = 5;

% Baut den Roboter und minJerkPath
KSetUp;
KPfadgenerator;

% input der achsstellungen
init_ax_values = [ones(size(minJerkPath,1),1) minJerkPath];
init_ax_values(:,1) = 0.3; %Anfängliche Beschleunigung = 0,3m/s???

% Defineig options für das Optimalsteuerungsprob.
opts = optimoptions(@fmincon, ...
    'Algorithm','interior-point', ...
    "MaxFunctionEvaluations",500000,...
    "MaxIterations",1000, ...
    "StepTolerance",1e-6, ...
    "OptimalityTolerance",1e-6, ...
    "EnableFeasibilityMode",true, ...
    "SubproblemAlgorithm",'factorization', ...
    "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
    "Display",'iter','ConstraintTolerance',10);

% Definig ub lb and options
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
min_values = repmat(cat(2,[0],min_jointangle),[size(init_ax_values,1),1]); 
max_values = repmat(cat(2,[5],max_jointangle),[size(init_ax_values,1),1]);

problem = createOptimProblem('fmincon',...
    'x0',init_ax_values, ...
    'objective',@optimization_task,...
    'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,regression,simulation_data), ...
    'lb',min_values,...
    'ub',max_values, ...
    'options',opts);
    
%[x,fval,eflag,output] = fmincon(problem);
ms = MultiStart;
[x,f] = run(ms,problem,1);

%Plotting output
show_spline(x, 'Bad Output');

function objective = optimization_task(optimization_values)
    timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
    base_points = optimization_values(:,2:size(optimization_values,2));
    objective = sum(timeintervals); 
    for i=1:size(base_points,2)
        [t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
        %0.0001
        objective = objective + 0.0001 * trapz((tdd.^2)); %TODO 
    end
end

        