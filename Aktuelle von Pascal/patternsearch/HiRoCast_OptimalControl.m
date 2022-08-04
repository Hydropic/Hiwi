close all
%Variablen
regression = load("regressionKat4.mat");
simulation_data = readtable('simulationData.txt');

lb = [];
ub = [];

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


% Definig ub lb and options
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
 
max_values = repmat(cat(2,[5],max_jointangle),[size(init_ax_values,1),1]);
min_values = repmat(cat(2,[0],min_jointangle),[size(init_ax_values,1),1]);


rng default % For reproducibility
nvars = length(minJerkPath())*7;
% set lb and ub
% TODO: lb und UB für Achswinkelgrenzen setzen
for i = 1:7 
    if i == 1
        for e = 1:length(minJerkPath())
            lb(end+1) = 0.05;
            ub(end+1) = 1;
        end
    else
        for e = 1:length(minJerkPath())
            lb(end+1) = -2*pi*2;
            ub(end+1) = 2*pi*2;
        end
    end
end

% Pass fixed parameters to objfun
objfun4 = @(x)optimization_task(x);

% Set nondefault solver options
options4 = optimoptions("ga","Display","diagnose","PlotFcn",["gaplotdistance",...
    "gaplotscores","gaplotstopping","gaplotmaxconstr","gaplotbestf",...
    "gaplotbestindiv","gaplotrange"]);

% Solve
[solution,objectiveValue] = patternsearch(objfun4,nvars,[],[],[],[],lb,ub,@(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,regression,simulation_data),...
    [],options4);

%Plotting output
show_spline(x, 'Bad Output');

function objective = optimization_task(optimization_values)

    optimization_values = transformValuseToMatrix(optimization_values);
    timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
    base_points = optimization_values(:,2:size(optimization_values,2));
    objective = sum(timeintervals); 
    for i=1:size(base_points,2)
        objective = objective 
    end
end

        