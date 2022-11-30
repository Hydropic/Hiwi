function [c,ceq] = constraintFcnValidation_A6(optimization_values, optimized_translational_values_oriented)


    % velocity GRAD/SEC
    max_velocity = deg2rad(100);
    min_velocity = deg2rad(-100);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad(200);
    min_acceleration = deg2rad(-200);
    ceq =[];
    c=[];
    
    [path,veolocity,acceleration,jerk,timesSteps,splinePunkt] = splineOptimal(optimization_values,optimized_translational_values_oriented(1:end-1,1),false);
    
    % Geschwindigkeit und Beschleunigung am Anfang muss 0 sein
    ceq(end+1) = veolocity(splinePunkt(1));
    ceq(end+1) = veolocity(splinePunkt(end));
    ceq(end+1) = acceleration(splinePunkt(1));
    ceq(end+1) = acceleration(splinePunkt(end));
    
    % BASEPOINTS BOUNDS 720 c's          
     for j=1:length(optimization_values) %for every point on every w
        c(end+1) = veolocity(splinePunkt(j)) - max_velocity(1);%velocity
        c(end+1) = min_velocity(1) - veolocity(splinePunkt(j));
        c(end+1) = acceleration(splinePunkt(j)) - max_acceleration(1);%acceleration
        c(end+1) = min_acceleration(1) - acceleration(splinePunkt(j)); 
     end  

%% =========Ausgabe auf der Konsole=============================     
    c_string = join(string( find(c > 0)   ), ',');   
    position_c_fail = find(c > 0);
    c_value_string = [];
    %c_value_string = join(string( find(c > 0)   ), ',');
    for z = 1: length(position_c_fail)
        c_value_string(end+1,1) = c(position_c_fail(z));
    end    
    %ausgabe der anzah der c Bedingungen und die anzahl der verletzten
    if ismissing(c_string) == false
        fprintf('c(%d, %d) [%s] \n',length(c),length(find(c-1e-6 > 0)) ,strjoin(string(round(c_value_string,1)),', '));
    else
        fprintf('Keine Bedingung verletzt')
        fprintf(num2str(length(c)));       
    end
   
end