close all
%Variablen
regression = load("regressionKat4.mat");
simulation_data = readtable('simulationData.txt');

%Load data
% test = load('optimized_input.mat')
% init_values = output.bestfeasible.x;
% init_values(:,1) = 5;

% Baut den Roboter und minJerkPath
KSetUp;
KPfadgenerator;

% input der achsstellungen
init_ax_values = [ones(size(minJerkPath,1),1) minJerkPath];
init_ax_values(:,1) = 0.3; %Anfängliche Beschleunigung = 0,3m/s???

opts = optimoptions('patternsearch',...
     'Display', 'iter', ...
     'UseParallel', true,...
     'MaxFunctionEvaluations',9000,...
     'StepTolerance', 1e-6,...
     'InitialMeshSize', 100,...
     'AccelerateMesh', true,...
     'PlotFcn',["psplotbestf","psplotmeshsize","psplotfuncount","psplotbestx","psplotmaxconstr"], 'ConstraintTolerance',0.1);

% Definig ub lb and options
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
min_values = repmat(cat(2,[0],min_jointangle),[size(init_ax_values,1),1]); 
max_values = repmat(cat(2,[0.5],max_jointangle),[size(init_ax_values,1),1]);

[eins, zwei] = size(init_ax_values);
for i = 1:eins-1
    for a = 1:zwei
        init_ax_values(i,a) = 0.5;
    end
end

[x,fval,eflag,output] = patternsearch(@optimization_task, init_ax_values, [], [], [],[], min_values, max_values, @(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,regression,simulation_data), opts);


%Plotting output
show_spline(x, 'Bad Output');
KSetUp;
showBahn = x;
showBahn(:,1) = [];
laufbahn(robot,showBahn,1,true)

function objective = optimization_task(optimization_values)
    timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
    base_points = optimization_values(:,2:size(optimization_values,2));
    objective = sum(timeintervals); 
    for i=1:size(base_points,2)
        %[t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
        %0.0001
        objective = objective;
        %+ 0.0001 * trapz((tdd.^2)); %TODO 
    end
end

        