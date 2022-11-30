%Close all open figures
close all

%multistart method: starts one optim. wihtout validation-> better input
%path to optim with validation (validation=sloshing/schwappen)
% [x,fval,eflag,output] = multistart_optim(10, 0.3);

% manual setup of one optimation
KSetUp;
%Optimizing minJerkPath from Pfadgenerator without Validation 
numSamples = 20;

KPfadgenerator;

% input
init_values = [ones(size(minJerkPath,1),1) minJerkPath];
init_values(:,1) = 0.3;


%Load data
% load('optimized_input_x.mat')
% init_values = output.bestfeasible.x;
% init_values(:,1) = 5;

% Defineig options
opts = optimoptions(@fmincon, ...
    'Algorithm','interior-point', ...
    "MaxFunctionEvaluations",500000,...
    "MaxIterations",1000, ...
    "StepTolerance",1e-6, ...
    "OptimalityTolerance",1e-6, ...
    "EnableFeasibilityMode",true, ...
    "SubproblemAlgorithm",'factorization', ...
    "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
    "Display",'iter', 'UseParallel', true,'ConstraintTolerance',120);

%opts = optimoptions('ga');
% "EnableFeasibilityMode",true, ...
%    "EnableFeasibilityMode",true, ...

% Definig ub lb and options
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
min_values = repmat(cat(2,[0],min_jointangle),[size(init_values,1),1]); 
max_values = repmat(cat(2,[5],max_jointangle),[size(init_values,1),1]); 

problem = createOptimProblem('fmincon',...
    'x0',init_values, ...
    'objective',@optimization_task,...
    'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_values), ...
    'lb',min_values,...
    'ub',max_values, ...
    'options',opts);
problem = createOptimProblem('interior-point',...
'x0',init_values, ...
'objective',@optimization_task,...
'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_values), ...
'lb',min_values,...
'ub',max_values, ...
'options',opts);
%gs = GlobalSearch;
% % % % gs.NumTrialPoints = 10;
% % % % gs.NumStageOnePoints = 10;
% % % % gs.Display = 'iter';
%%%x = ga(@optimization_task,7,[],[],[],[],[],[],@constraintFcnValidation,opts);

[x,fval,eflag,output] = fmincon(problem);
%[x,f] = run(gs,problem);

%Plotting output
if ~isempty(output.bestfeasible)
    show_spline(output.bestfeasible.x, 'Good Output');
else
    show_spline(x, 'Bad Output');
end

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

function [c,ceq] = constraintFcnValidation(optimization_values, input_backup)
% Example: Constrain a solution to the region
% x^2 + y^2 <= 5
% x^2 + y^2 >= 2
% y = x^3
% Note, if no inequality constraints, specify c = []
% Note, if no equality constraints, specify ceq = []
% c(1) = 1^2 + 1^2 - 5;
% c(2) = 2 - 1^2 - 1^2;
x = 0;

% joint angle

max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
% velocity GRAD/SEC
max_velocity = deg2rad([400,400,400,400,400,400]);
min_velocity = deg2rad([-400,-400,-400,-400,-400,-400]);
% acceleration GRAD/SEC2
max_acceleration = deg2rad([200,200,200,200,200,200]);
min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);
% jerk deg/s^3
max_jerk = deg2rad([1000,1000,1000,1000,1000,1000]);
min_jerk = deg2rad([-1000,-1000,-1000,-1000,-1000,-1000]);
% Unzip optimization input
timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
base_points = optimization_values(:,2:size(optimization_values,2));
input_backup = input_backup(:,2:size(optimization_values,2));

%Precision of equality constraints modelled as inequality constraints
precision = 0.01; %0.057296°

%init
ceq=[];
c=[];

% TIMEINTERVALS > 0
c(end+1:end+length(timeintervals))= 0-timeintervals; %Warum?
for t = 1:length(c)
    if c(1,t) > 0
            fprintf(['bedingung timeintervall',t,'verletzt']);
            
     end
end

% SLOSHING 
[v_c, v_ceq] = CompleteValidation(timeintervals, base_points);


if length(v_c) >= 1
    c(end+1:end+length(v_c)) = v_c;
end
if length(v_ceq) >= 1
    ceq(end+1:end+length(v_ceq)) = v_ceq;
end

% KEEP START AND END- POSITIONS (,1) wichtig damit vector und nicht array
[start_pos_backup, start_kelle_backup] = vorwaertskinematik(input_backup(1,:));
[start_pos_new, start_kelle_new] = vorwaertskinematik(base_points(1,:));
[end_pos_backup, end_kelle_backup] = vorwaertskinematik(input_backup(size(base_points,1),:));
[end_pos_new, end_kelle_new] = vorwaertskinematik(base_points(size(base_points,1),:));
for i=1:3
    ceq(end+1) = start_pos_backup(i) - start_pos_new(i);
    if ceq(end) > 0
            fprintf('Bedingung 10 Verletzt');
            fprintf(num2str(c(end)));
    end
    ceq(end+1) = end_pos_backup(i) - end_pos_new(i);
    if ceq(end) > 0 % einzelne fehler
            fprintf('Bedingung 11 Verletzt');
            fprintf(num2str(c(end)));
    end
    %TODO ohne kellen orientierung testen
    %ceq(end+1) = start_kelle_backup(i) - start_kelle_new(i);
    %ceq(end+1) = end_kelle_backup(i) - end_kelle_new(i);
end


for i=1:size(base_points,2) %for every axis
     [t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
     if 0 ~= sum(place==0)
        fprintf('ESKALATION/ERROR in place/spline');
        fprintf('place(%d) [%s] \n',length(place),join(string( place ), ','));
     end

     % KEEP VELOCITY 0 (START/END) %Keine probleme
% %      ceq(end+1) = td(place(1)) - 0; %Wichtig muss eventuell wider rein
% %      if ceq(end) > 0 % häufig fehler
% %             fprintf('Bedingung 12 Verletzt');
% %             fprintf(num2str(c(end)));
% %      end
% %      ceq(end+1) = td(place(size(base_points,1))) - 0;
% %      if ceq(end) > 0% häufug fehler
% %             fprintf('Bedingung 13 Verletzt');
% %             fprintf(num2str(c(end)));
% %      end
     c(end+1) = abs(td(place(1)) - 0) - precision;
     if c(end) > 0
            fprintf('Bedingung 14 Verletzt');
            fprintf(num2str(c(end)));
     end
     c(end+1) = abs(td(place(size(base_points,1))) - 0) - precision; 
     if c(end) > 0
            fprintf('Bedingung 15 Verletzt');
            fprintf(num2str(c(end)));
     end

     % KEEP START AND END- POINTS 
%      ceq(end+1) = base_points(1,i) - input_backup(1,i); %TODO ggf als kellenposition
%      ceq(end+1) = base_points(size(base_points,1),i) - input_backup(size(base_points,1),i);
    c(end+1) = abs(base_points(1,i) - input_backup(1,i)) - precision;
    c(end+1) = abs(base_points(size(base_points,1),i) - input_backup(size(base_points,1),i)) - precision;

     % ALL POINTS BOUNDS
% 	 ceq(end+1) = sum(t>max_jointangle(i))-0;%joint angle
%      ceq(end+1) = sum(t<min_jointangle(i))-0;
%      ceq(end+1) = sum(td>max_velocity(i))-0;%velocity
%      ceq(end+1) = sum(td<min_velocity(i))-0;
%      ceq(end+1) = sum(tdd>max_acceleration(i))-0;%acceleration
%      ceq(end+1) = sum(tdd<min_acceleration(i))-0;
%      ceq(end+1) = sum(tddd>max_jerk(i))-0;%jerk
%      ceq(end+1) = sum(tddd<min_jerk(i))-0;  
     
     % BASEPOINTS BOUNDS %Verletzt nicht die Bedingungen
     for j=1:size(base_points,1) %for every point on every axis
       c(end+1) = base_points(j,i) - max_jointangle(i);%joint angle 
       if c(end) > 0
            fprintf('Bedingung 16 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = min_jointangle(i) - base_points(j,i); 
       if c(end) > 0
            fprintf('Bedingung 17 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = td(place(j)) - max_velocity(i);%velocity
       if c(end) > 0
            fprintf('Bedingung 18 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = min_velocity(i) - td(place(j));
       if c(end) > 0
            fprintf('Bedingung 19 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = tdd(place(j)) - max_acceleration(i);%acceleration
       if c(end) > 0
            fprintf('Bedingung 20 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = min_acceleration(i) - tdd(place(j));
       if c(end) > 0
            fprintf('Bedingung 21 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = tddd(place(j)) - max_jerk(i);%jerk
       if c(end) > 0
            fprintf('Bedingung 22 Verletzt');
            fprintf(num2str(c(end)));
       end
       c(end+1) = min_jerk(i) - tddd(place(j));
       if c(end) > 0
            fprintf('Bedingung 23 Verletzt');
            
       end

     end
end



ceq_string = join(string( find(ceq ~= 0)   ), ',');
c_string = join(string( find(c > 0)   ), ',');
if ismissing(c_string) == false 
    fprintf('c(%d, %d) [%s] \n',length(c),length(find(c-1e-6 > 0)) ,c_string);
    
end
if ismissing(ceq_string) == false 
    fprintf('ceq(%d, %d) [%s] \n',length(ceq),length(find(ceq ~= 0)) ,ceq_string);
   
end

fprintf('###########################################################################################\n');
end


function show_spline(solution, window_title)
timeintervals = solution(1:size(solution,1)-1,1);
base_points = solution(:,2:size(solution,2));
figure('name', window_title);
for i=1:size(base_points,2)
    fprintf('SPLINE %d',i);
    [t,td,tdd,tddd,time,place] = spline(base_points(:,i)',timeintervals,false);
    
    subplot(4,size(base_points,2),1+(i-1))
    hold on
    plot(compute_time(timeintervals),base_points(:,i), "linestyle", "none","marker","o");
    plot(time,t, "Color","green");
    title("Spline ",i);
    xlabel("Time");
    ylabel("Angle");
    ax = gca;
    ax.YAxis.Exponent = 0;
    hold off
 
    %Plotten der 1. Ableitung/Geschwindigkeit
    subplot(4,size(base_points,2),7+(i-1))
    plot(time,td, "Color","magenta");
    title("Velocity");
    xlabel("Time");
    ylabel("Velocity");
    ax = gca;
    ax.YAxis.Exponent = 0;
    
    %Plotten der 2. Ableitung/Beschleunigung 
    subplot(4,size(base_points,2),13+(i-1))
    hold on 
    plot(time,tdd, "Color","yellow");
    title("Acceleration");
    xlabel("Time");
    ylabel("Acceleration");
    %Plottet nur die Ableitungen der Punkte 
    %plot(ti,[0,ableitungen2,0],"marker","o");
    ax = gca;
    ax.YAxis.Exponent = 0;
    hold off 
    
    %Plotten der 3. Ableitung/Ruck
    subplot(4,size(base_points,2),19+(i-1)) 
    plot(time,tddd, "Color","red");
    title("Jerk");
    xlabel("Time");
    ylabel("Jerk");
    ax = gca;
    ax.YAxis.Exponent = 0;

end
fprintf('Zeit: %f',sum(timeintervals))
end

function array = compute_time(intervals)
array(1)=0;
for i=2:length(intervals)+1;
    array(i)= sum(intervals(1:i-1));
end
end




