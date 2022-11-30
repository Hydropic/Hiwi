function [c,ceq] = constraintFcnValidation_orientationY(optimization_values, optimized_translational_values_oriented)

%% =========Feste Variablen==========================================
    % velocity GRAD/SEC
    max_velocity = deg2rad([80,75,70,70,70,100]);
    min_velocity = deg2rad([-80,-75,-70,-70,-70,-100]);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad([200,200,200,200,200,200]);
    min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);

    jerkValue = 3000; % um richtig Power zu geben 100000
    % jerk deg/s^3
    max_jerk = deg2rad([jerkValue,jerkValue,jerkValue,jerkValue,jerkValue,jerkValue]);
    min_jerk = deg2rad([-jerkValue,-jerkValue,-jerkValue,-jerkValue,-jerkValue,-jerkValue]);

    %init
    ceq=[];
    c=[];
    optimized_translational_values_TCP_orientation = [];

    % Unzip optimization input
    base_Zeitintervall = optimized_translational_values_oriented(1:size(optimized_translational_values_oriented,1)-1,1);
    base_Achswinkel = optimized_translational_values_oriented(:,2:size(optimized_translational_values_oriented,2));
    base_Achswinkel(:,6) = optimization_values(:,1);        
    
%% =========Kontrollen der einzelnen Achsgrenzen==============
for w = 6:6
         [path,veolocity,acceleration,jerk,timesSteps,splinePunkt] = splineOptimal(base_Achswinkel(:,w),base_Zeitintervall,false);

          % Geschwindigkeit und Beschleunigung am Anfang muss 0 sein
          ceq(end+1) = veolocity(splinePunkt(1));
          ceq(end+1) = veolocity(splinePunkt(end));
          ceq(end+1) = acceleration(splinePunkt(1));
          ceq(end+1) = acceleration(splinePunkt(end));
    
         % BASEPOINTS BOUNDS 720 c's          
         for j=1:size(base_Achswinkel,1) %for every point on every w
            c(end+1) = veolocity(splinePunkt(j)) - max_velocity(w);%velocity
            c(end+1) = min_velocity(w) - veolocity(splinePunkt(j));
            c(end+1) = acceleration(splinePunkt(j)) - max_acceleration(w);%acceleration
            c(end+1) = min_acceleration(w) - acceleration(splinePunkt(j)); 
%            c(end+1) = jerk(splinePunkt(j)) - max_jerk(w);%jerk
%            c(end+1) = min_jerk(w) - jerk(splinePunkt(j));  
         end            
end
    
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