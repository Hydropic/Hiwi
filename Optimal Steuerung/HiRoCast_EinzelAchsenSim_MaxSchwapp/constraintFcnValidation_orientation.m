function [c,ceq] = constraintFcnValidation_orientation(optimized_translational_values_5_6,init_optimization_values,optimized_translational_values, orientationsPendulum)
   

optimized_translational_values_5_6(:,1) = [];
optimized_translational_values_5_6(:,2) = [];
optimized_translational_values_5_6(:,3) = [];
optimized_translational_values_5_6(:,4) = [];
% Update all valuse
[optimized_translational_values] = updateTimeAndAxes(optimized_translational_values_5_6, optimized_translational_values);

%% =========Feste Variablen==========================================
    % velocity GRAD/SEC
    max_velocity = deg2rad([400,400,400]);
    min_velocity = deg2rad([-400,-400,-400]);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad([200,200,200]);
    min_acceleration = deg2rad([-200,-200,-200]);
    [eins, zwei] = size(optimized_translational_values);  

    % jerk deg/s^3
    max_jerk = deg2rad([1000,1000,1000,1000,1000,1000]);
    min_jerk = deg2rad([-1000,-1000,-1000,-1000,-1000,-1000]);

    %init
    ceq=[];
    c=[];
    optimized_translational_values_TCP_orientation = [];

    % Unzip optimization input
    base_Zeitintervall = optimized_translational_values(1:size(optimized_translational_values,1)-1,1);
    base_Achswinkel = optimized_translational_values(:,2:size(optimized_translational_values,2));
    %initial_AchsStellung = initial_AchsStellung(:,2:size(optimization_values,2));     
        
%% =========Kontrollen der einzelnen Achsgrenzen==============
    for i=1:size(optimized_translational_values_5_6,2) 
         [~,veolocity,acceleration,jerk,~,splinePunkt] = splineOptimal(optimized_translational_values_5_6(:,i),base_Zeitintervall,false);
            
          %Geschwindigkeit am Anfang muss 0 sein
          ceq(end+1) = veolocity(splinePunkt(1));
          ceq(end+1) = veolocity(splinePunkt(end));    
          ceq(end+1) = acceleration(splinePunkt(1));
          ceq(end+1) = acceleration(splinePunkt(end));  
          
         %BASEPOINTS BOUNDS 720 c's          
             for j=1:size(optimized_translational_values_5_6,1) %for every point on every axis
                c(end+1) = (veolocity(splinePunkt(j)) - max_velocity(i))*1;%velocity
                c(end+1) = (min_velocity(i) - veolocity(splinePunkt(j)))*1;
                c(end+1) = (acceleration(splinePunkt(j)) - max_acceleration(i))*1;%acceleration
                c(end+1) = (min_acceleration(i) - acceleration(splinePunkt(j)))*1; 
                c(end+1) = jerk(j) - max_jerk(i);%jerk
                c(end+1) = min_jerk(i) - jerk(j);  
             end            
    end

    %% =========Kontrollen der Pendellunterschiede==============
    % split valuse anguleTCP = [z, y', x'']
    [position_TCP, angleTCP, optimization_values] = prepairValuse_5_6(optimized_translational_values, 99);

    % TODO: die x und Y Ebene umrechenen von der Pendelbewegung auf die aktuelle im TCP (abhängig von z Rotation der Kelle)
    orientationsPendulum(:,3) = -orientationsPendulum(:,3)
    [orientationsPendulum_XY_timeCorrected] = matchPendelTimeSteps(orientationsPendulum, optimized_translational_values);
    angularDifference = [];
    acceptableOrientationDifference = 1 %°

    %Vergleich der Orientierung der Kelle mit der Auslenkung des Pendels
    for i = 1:length(orientationsPendulum_XY_timeCorrected)
        angularDifference(end+1) = orientationsPendulum_XY_timeCorrected(i, 3) - angleTCP(i, 4);
        angularDifference(end+1) = orientationsPendulum_XY_timeCorrected(i, 2) - angleTCP(i, 3);
        c(end+1) = angleTCP(i, 3) - orientationsPendulum_XY_timeCorrected(i, 2) - acceptableOrientationDifference;
        c(end+1) = orientationsPendulum_XY_timeCorrected(i, 2) + acceptableOrientationDifference - angleTCP(i, 3);

        c(end+1) = angleTCP(i, 4) - orientationsPendulum_XY_timeCorrected(i, 3) - acceptableOrientationDifference;
        c(end+1) = orientationsPendulum_XY_timeCorrected(i, 3) + acceptableOrientationDifference - angleTCP(i, 4);
%         ceq(end+1) = orientationsPendulum_XY_timeCorrected(i, 2) - angleTCP(i, 4);
%         ceq(end+1) = orientationsPendulum_XY_timeCorrected(i, 3) - angleTCP(i, 3);
    end

    % Saving the angular difference between pendulum and trowel orientation
    sumOrientationDifference = sum(abs(angularDifference));
    save('sum_Orientation_Difference.mat','sumOrientationDifference');
    
%     % Update all valuse
%     [optimized_translational_values_update] = updateTimeAndAxes(optimization_values, optimized_translational_values)

%% =========Ausgabe auf der Konsole=============================     
    ceq_string = join(string( find(ceq ~= 0)   ), ',');
    c_string = join(string( find(c > 0)   ), ',');   
    position_c_fail = find(c > 0);
    c_value_string = [];
    %c_value_string = join(string( find(c > 0)   ), ',');
    for z = 1: length(position_c_fail)
        c_value_string(end+1,1) = c(position_c_fail(z));
    end
    position_ceq_fail = find(ceq > 0); 
    ceq_value_string = [];
    %c_value_string = join(string( find(c > 0)   ), ',');
    for z = 1: length(position_ceq_fail)
        ceq_value_string(end+1,1) = ceq(position_ceq_fail(z));
    end

    %ausgabe der anzah der c Bedingungen und die anzahl der verletzten
    if ismissing(c_string) == false
        fprintf('c(%d, %d) [%s] \n',length(c),length(find(c-1e-6 > 0)) ,strjoin(string(round(c_value_string,1)),', '));
    else
        fprintf('Keine Bedingung verletzt')
        fprintf(num2str(length(c)));       
    end
    if ismissing(ceq_string) == false  
        fprintf('ceq(%d, %d) [%s] \n',length(ceq),length(find(ceq-1e-6 > 0)) ,strjoin(string(round(ceq_value_string,1)),', '));    
    end  
end