%Close all open figures
close all
workspace
%multistart method: starts one optim. wihtout validation-> better input
%path to optim with validation (validation=sloshing/schwappen)
% [x,fval,eflag,output] = multistart_optim(10, 0.3);

% manual setup of one optimation
KSetUp;

%Optimizing minJerkPath from Pfadgenerator without Validation 
numSamples = 7;
KPfadgenerator;

% input
init_values = [ones(size(minJerkPath,1),1) minJerkPath];
init_values(:,1) = 1;




%Load data
%load('optimized_input_x.mat')
%init_values = output.bestfeasible.x;
%init_values(:,1) = 0.01;

% Defineig options
opts = optimoptions(@fmincon, ...
    'Algorithm','interior-point', ...
    "MaxFunctionEvaluations",500000,...
    "MaxIterations",1000, ...
    "StepTolerance",1e-10, ...
    "OptimalityTolerance",1e-6, ...
    "SubproblemAlgorithm",'factorization', ...
    "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
    "Display",'final-detailed');
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
    
[x,fval,eflag,output] = fmincon(problem);

% Plotting output
if ~isempty(output.bestfeasible)
    show_spline(output.bestfeasible.x, 'Good Output');
else
    show_spline(x, 'Bad Output');
end

% Method: multistart
function [x,fval,eflag,output] = multistart_optim(nBasePoints, startintervals)

% Defineig options
opts = optimoptions(@fmincon, ...
    'Algorithm','interior-point', ...
    "MaxFunctionEvaluations",500000,...
    "MaxIterations",500, ...
    "StepTolerance",1e-9, ...
    "OptimalityTolerance",1e-4, ...
    "EnableFeasibilityMode",true, ...
    "SubproblemAlgorithm",'factorization', ...
    "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
    "Display",'final-detailed');
%"PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
% "Diagnostics","on", ...

fprintf('######### Run w.o. Validation with %d points and %d steps ######################\n',nBasePoints, startintervals);
%Generate path:
KSetUp;

%Optimizing minJerkPath from Pfadgenerator without Validation 
numSamples = nBasePoints;
KPfadgenerator;

init_values = [ ones(size(minJerkPath,1),1) minJerkPath];
init_values(:,1) = startintervals;

% Definig ub lb and options
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
min_values = repmat(cat(2,[0],min_jointangle),[size(init_values,1),1]); 
max_values = repmat(cat(2,[10],max_jointangle),[size(init_values,1),1]); 

problem = createOptimProblem('fmincon',...
    'x0',init_values, ...
    'objective',@optimization_task,...
    'nonlcon', @(optimization_values)constraintFcn(optimization_values,init_values), ...
    'lb',min_values,...
    'ub',max_values, ...
    'options',opts);

[x,fval,eflag,output] = fmincon(problem);

% Optimizing first Optimizations output with Validation TODO nur input
% variable setzen anstelle mehrfach definitionen
if ~isempty(output.bestfeasible)
    x = output.bestfeasible.x;
    %x(:,1) = startintervals;
end

problemValidation = createOptimProblem('fmincon',...
    'x0',x, ...
    'objective',@optimization_task,...
    'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_values), ...
    'lb',min_values,...
    'ub',max_values, ...
    'options',opts);

fprintf('######### Run w. Validation with %d points ######################\n',nBasePoints);
[x,fval,eflag,output] = fmincon(problemValidation);
end 

function objective = optimization_task(optimization_values)
timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
base_points = optimization_values(:,2:size(optimization_values,2));
objective = sum(timeintervals); 
for i=1:size(base_points,2)
    [t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
    %0.0001
    %objective = objective + 0.0001 * trapz((tdd.^2)); %TODO 
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


 %% Regression for Schwapp
    
%  %load("regressionBeschleuinigung.mat");

%  %Range = 50;
%  %kelle_neu = kelle_orientierung_beschleunigung_neu(1:6);
%  %kelle_neu(7) = phi_1;
%  kelle_neu(8) = phi_2;
%  newFit = trainedModel.predictFcn(kelle_neu);
% 
%  DifferenzSwap = abs(newFit) - abs(kelle_orientierung_beschleunigung_neu(7));
% 
%  if DifferenzSwap > Range/2
%     disp("Such bessere!");
%  end
% 
%  if DifferenzSwap < -Range/2
%      disp("Such bessere!");
%  end


%%%%%%%%%%%%%%%%%%%%%%%

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
precision = 0.0000001; %0.0000057296°
%TEST LPS: timeintervals = [-1,-2,1000,1000,1000,1000];
%init
ceq=[];
c=[];

% TIMEINTERVALS > 0
c(end+1:end+length(timeintervals))= 0-timeintervals; 

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
% for i=1:3
%     ceq(end+1) = start_pos_backup(i) - start_pos_new(i);
%     ceq(end+1) = end_pos_backup(i) - end_pos_new(i);
%     %TODO ohne kellen orientierung testen
%     %ceq(end+1) = start_kelle_backup(i) - start_kelle_new(i);
%     %ceq(end+1) = end_kelle_backup(i) - end_kelle_new(i);
% end


for i=1:size(base_points,2) %for every axis
     [t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
     if 0 ~= sum(place==0)
        fprintf('ESKALATION/ERROR in place/spline');
        fprintf('place(%d) [%s] \n',length(place),join(string( place ), ','));
     end

     % KEEP VELOCITY 0 (START/END)
     ceq(end+1) = td(place(1)) - 0;
     ceq(end+1) = td(place(size(base_points,1))) - 0;

     % KEEP START AND END- POINTS 
     ceq(end+1) = base_points(1,i) - input_backup(1,i); %TODO ggf als kellenposition
     ceq(end+1) = base_points(size(base_points,1),i) - input_backup(size(base_points,1),i);
     
     % ALL POINTS BOUNDSh
	 % Will be deleted: ceq(end+1) = sum(t>max_jointangle(i))-0;%joint angle
     %ceq(end+1) = sum(t<min_jointangle(i))-0;
     %ceq(end+1) = sum(td>max_velocity(i))-0;%velocity
     %ceq(end+1) = sum(td<min_velocity(i))-0;
     %ceq(end+1) = sum(tdd>max_acceleration(i))-0;%acceleration
     %ceq(end+1) = sum(tdd<min_acceleration(i))-0;
     %ceq(end+1) = sum(tddd>max_jerk(i))-0;%jerk
     %ceq(end+1) = sum(tddd<min_jerk(i))-0;  
     
     % BASEPOINTS BOUNDS
     for j=1:size(base_points,1) %for every point on every axis
        c(end+1) = base_points(j,i) - max_jointangle(i);%joint angle
        c(end+1) = min_jointangle(i) - base_points(j,i);
        c(end+1) = td(place(j)) - max_velocity(i);%velocity
        c(end+1) = min_velocity(i) - td(place(j));
        c(end+1) = tdd(place(j)) - max_acceleration(i);%acceleration
        c(end+1) = min_acceleration(i) - tdd(place(j));   
        c(end+1) = tddd(place(j)) - max_jerk(i);%jerk
        c(end+1) = min_jerk(i) - tddd(place(j));   
     end
end

simu_CFD = importdata('simulationData.txt');

orientierung_mesh = simu_CFD(:,1:3);
    
kelle_orientierung_beschleunigung_neu = [12.7 8.9 0 0.34963 -0.35744 0 -4.11e+03]; % Rx Ry Rz nx ny nz A
kelle_orientierung = kelle_orientierung_beschleunigung_neu(:,1:3);
kelle_richtung_neu = kelle_orientierung_beschleunigung_neu(:,4:6);

[k,dist] = dsearchn(orientierung_mesh,kelle_orientierung);

simu_CFD_id_k = simu_CFD(k,:);

vorzugsBewegungsrichtung = simu_CFD_id_k(:,4:6);

 phi_1 = CalculateAngleInXYPlane(vorzugsBewegungsrichtung,kelle_richtung_neu);

 phi_2 = 90 - (acos(dot([0 0 1],kelle_richtung_neu)/(norm(kelle_richtung_neu)))/pi * 180);

 simu_CFD_phi1_k9 = simu_CFD(k:(k+8),7);
 simu_CFD_phi2_k9 = simu_CFD(k:(k+8),8);

 phi1_min = min(simu_CFD_phi1_k9);
 phi1_max = max(simu_CFD_phi1_k9);


 phi2_min = min(simu_CFD_phi2_k9);
 phi2_max = max(simu_CFD_phi2_k9);

% if ~((phi1_min < phi_1) && (phi_1 < phi1_max))
   % fprintf('Angle not in corridor');
% end

% if ~((phi2_min < phi_2) && (phi_2 < phi2_max))
  %  fprintf('Angle not in corridor');
% end

c(end+1) = phi_1 - phi1_max;
c(end+1) = phi1_min - phi_1;
c(end+1) = phi_2 - phi2_max;
c(end+1) = phi2_min - phi_2;

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

function [c,ceq] = constraintFcn(optimization_values, input_backup)
% Example: Constrain a solution to the region
% x^2 + y^2 <= 5
% x^2 + y^2 >= 2
% y = x^3
% Note, if no inequality constraints, specify c = []
% Note, if no equality constraints, specify ceq = []
% c(1) = 1^2 + 1^2 - 5;
% c(2) = 2 - 1^2 - 1^2;

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
precision = 0.0000001; %0.0000057296°

%init
ceq=[];
c=[];

% TIMEINTERVALS > 0
c(end+1:end+length(timeintervals))= 0-timeintervals; 

% SLOSHING
[v_c, v_ceq] = HiRoCast_Validation(base_points, timeintervals, true);
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
    ceq(end+1) = end_pos_backup(i) - end_pos_new(i);

    ceq(end+1) = start_kelle_backup(i) - start_kelle_new(i);
    ceq(end+1) = end_kelle_backup(i) - end_kelle_new(i);
end

for i=1:size(base_points,2) %for every axis
     [t,td,tdd,tddd,time,place] = spline(base_points(:,i),timeintervals,false);
     if 0 ~= sum(place==0)
        fprintf('ESKALATION/ERROR in place/spline');
        fprintf('place(%d) [%s] \n',length(place),join(string( place ), ','));
     end

     % KEEP VELOCITY 0 (START/END)
     ceq(end+1) = td(place(1)) - 0;
     ceq(end+1) = td(place(size(base_points,1))) - 0;

     % KEEP START AND END- POINTS 
     ceq(end+1) = base_points(1,i) - input_backup(1,i); %TODO ggf als kellenposition
     ceq(end+1) = base_points(size(base_points,1),i) - input_backup(size(base_points,1),i);

     
     % ALL POINTS BOUNDS
	 ceq(end+1) = sum(t>max_jointangle(i))-0;%joint angle
     ceq(end+1) = sum(t<min_jointangle(i))-0;
     ceq(end+1) = sum(td>max_velocity(i))-0;%velocity
     ceq(end+1) = sum(td<min_velocity(i))-0;
     ceq(end+1) = sum(tdd>max_acceleration(i))-0;%acceleration
     ceq(end+1) = sum(tdd<min_acceleration(i))-0;
     ceq(end+1) = sum(tddd>max_jerk(i))-0;%jerk
     ceq(end+1) = sum(tddd<min_jerk(i))-0;  
     
     % BASEPOINTS BOUNDS
     for j=1:size(base_points,1) %for every point on every axis
        c(end+1) = base_points(j,i) - max_jointangle(i);%joint angle
        c(end+1) = min_jointangle(i) - base_points(j,i);
        c(end+1) = td(place(j)) - max_velocity(i);%velocity
        c(end+1) = min_velocity(i) - td(place(j));
        c(end+1) = tdd(place(j)) - max_acceleration(i);%acceleration
        c(end+1) = min_acceleration(i) - tdd(place(j));   
        c(end+1) = tddd(place(j)) - max_jerk(i);%jerk
        c(end+1) = min_jerk(i) - tddd(place(j));   
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




