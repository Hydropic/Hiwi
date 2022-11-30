close all
%Variablen
regression = load("regressionKat4.mat");
simulation_data = readtable('simulationData.txt');

%Load data
% load('optimized_input.mat')
% init_values = output.bestfeasible.x;
% init_values(:,1) = 5;

% Baut den Roboter und minJerkPath
KSetUp;
KPfadgenerator;
%laufbahn(robot,minJerkPath,1,true)

% input der achsstellungen
init_ax_values = [ones(size(minJerkPath,1),1) minJerkPath];
init_ax_values(:,1) = 0.1; %Anfängliche Beschleunigung = 0,3m/s???

% Defineig options für das Optimalsteuerungsprob.
opts = optimoptions(@fmincon, ...
    'Algorithm','interior-point', ...
    "MaxFunctionEvaluations",200000,...
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
min_values = repmat(cat(2,[0],min_jointangle),[size(init_ax_values,1),1]); 
max_values = repmat(cat(2,[0.2],max_jointangle),[size(init_ax_values,1),1]);

[eins, zwei] = size(max_values);
% Fix Start and End Position
for i = 1:zwei-1
    min_values(1, i+1) = minJerkPath(1, i);
    max_values(1, i+1) = minJerkPath(1, i);
    min_values(end, i+1) = minJerkPath(end, i);
    max_values(end, i+1) = minJerkPath(end, i);
end

% Fix Achs 1, 2 und 3
for i = 1:eins
    min_values(i, 5) = minJerkPath(1, 4);
    max_values(i, 5) = minJerkPath(1, 4);
    min_values(i, 6) = minJerkPath(1, 5);
    max_values(i, 6) = minJerkPath(1, 5);
    min_values(i, 7) = minJerkPath(1, 6);
    max_values(i, 7) = minJerkPath(1, 6);
end




% [eins, zwei] = size(init_ax_values);
% for i = 1:eins-1
%     for a = 1:zwei
%         init_ax_values(i,a) = 0.1;
%     end
% end

problem = createOptimProblem('fmincon',...
    'x0',init_ax_values, ...
    'objective',@optimization_task,...
    'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,regression,simulation_data), ...
    'lb',min_values,...
    'ub',max_values, ...
    'options',opts);
    
%[x,fval,eflag,output] = fmincon(problem);


%gs = GlobalSearch( 'FunctionTolerance' ,2e-4, 'NumTrialPoints' ,2000, 'UseParallel' ,true)


ms = MultiStart('UseParallel',true);
[x,f] = run(ms,problem,1);




KSetUp;
showBahn = x;
showBahn(:,1) = [];
laufbahn(robot,showBahn,1,true)


%Plotting output
show_spline(x, 'Bad Output');


function objective = optimization_task(optimization_values)
    timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
    base_points = optimization_values(:,2:size(optimization_values,2));
    objective = sum(timeintervals); 
    for i=1:size(base_points,2)
        %[achsstellung,vilocity,acceleration,yerk,time,splinePunkt] = spline(base_points(:,i),timeintervals,false);
        %0.0001
        objective = objective;
        %+ 0.00001 * trapz((vilocity.^2)); %TODO 
    end
end

        