function [c,ceq] = constraintFcnValidation(optimization_values, orientationsPendulum,callFrom)
        
%% =========Feste Variablen==========================================
    % velocity GRAD/SEC
    max_velocity = deg2rad([400,400,400,400,400,400]);
    min_velocity = deg2rad([-400,-400,-400,-400,-400,-400]);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad([200,200,200,200,200,200]);
    min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);

    %init
    ceq=[];
    c=[];
    optimized_translational_values_TCP_orientation = [];
    % Unzip optimization input
    base_Zeitintervall = optimization_values(1:size(optimization_values,1)-1,1);
    base_Achswinkel = optimization_values(:,2:size(optimization_values,2));
    %initial_AchsStellung = initial_AchsStellung(:,2:size(optimization_values,2));     
    
    
%% =========Kontrollen der einzelnen Achsgrenzen==============
    for i=1:size(base_Achswinkel,2) 
         [~,veolocity,acceleration,~,~,splinePunkt] = spline(base_Achswinkel(:,i),base_Zeitintervall,false);
            
          % Geschwindigkeit am Anfang muss 0 sein
          ceq(end+1) = veolocity(splinePunkt(1));
          ceq(end+1) = veolocity(splinePunkt(end));
    
         % BASEPOINTS BOUNDS 720 c's          
             for j=1:size(base_Achswinkel,1) %for every point on every axis
                c(end+1) = veolocity(splinePunkt(j)) - max_velocity(i);%velocity
                c(end+1) = min_velocity(i) - veolocity(splinePunkt(j));
                c(end+1) = acceleration(splinePunkt(j)) - max_acceleration(i);%acceleration
                c(end+1) = min_acceleration(i) - acceleration(splinePunkt(j));   
             end            
    end

    if callFrom == "pendel"   
        % Orientierung des TCP ermitteln
        for i= 1:row_initial_values
            p = [optimization_values(i, 2), optimization_values(i, 3), optimization_values(i, 4), optimization_values(i, 5), optimization_values(i, 6), optimization_values(i, 7)];
            [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p);
        
            optimized_translational_values_TCP_orientation(end+1,1) = optimization_values(i, 1);
            optimized_translational_values_TCP_orientation(end,2) = eulerXYZ(1);
            optimized_translational_values_TCP_orientation(end,3) = eulerXYZ(2);
            optimized_translational_values_TCP_orientation(end,4) = eulerXYZ(3);
        end    

        %Vergleich der Orientierung der Kelle mit der Auslenkung des Pendels
        for i = 1:length(optimized_translational_values_TCP)
            ceq(end+1) = orientationsPendulum(i, 2) - ptimized_translational_values_TCP_orientation(i, 2);
            ceq(end+1) = orientationsPendulum(i, 3) - ptimized_translational_values_TCP_orientation(i, 3);
            ceq(end+1) = orientationsPendulum(i, 4) - ptimized_translational_values_TCP_orientation(i, 4);
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