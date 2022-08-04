function [c,ceq] = constraintFcnValidation(optimization_values, initial_AchsStellung,regression,simulation_data)
   
   [optimization_values, conuterBasePoints, conuterTimeUAchses] = transformValuseToMatrix(optimization_values);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Feste Variablen%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % joint angle    
    max_jointangle = deg2rad([185,14,144,350,120,350]);
    min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);

    % velocity GRAD/SEC
    max_velocity = deg2rad([400,400,400,400,400,400]);
    min_velocity = deg2rad([-400,-400,-400,-400,-400,-400]);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad([200,200,200,200,200,200]);
    min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);

    %init
    ceq=[];
    c=[];
    regression_1 = regression;
    % Unzip optimization input
    base_Zeitintervall = optimization_values(1:size(optimization_values,1)-1,1);
    base_Zeitintervall_mod(1,1) = 0.1;%ggf. kleiner 0.1
    base_Zeitintervall_mod(2:size(base_Zeitintervall)+1,1) = base_Zeitintervall;
    base_Zeitintervall_mod(end+1,1) = 0.1;

    base_Achswinkel = optimization_values(:,2:size(optimization_values,2));
    base_Achswinkel_Mod(1,:) = initial_AchsStellung(1,2:7);
    base_Achswinkel_Mod(2:size(base_Achswinkel,1)+1,1:6) = base_Achswinkel;
    base_Achswinkel_Mod(end+1,:) = initial_AchsStellung(end,2:7);

    initial_AchsStellung = initial_AchsStellung(:,2:size(optimization_values,2));     
            
    %TIMEINTERVALS > 0 <0.5s Auch die virtuellen punkte Berücksichtigt
%     c(end+1:end+length(base_Zeitintervall_mod)) = 0-base_Zeitintervall_mod; 
%     c(end+1:end+length(base_Zeitintervall_mod)) = base_Zeitintervall_mod - 0.5;%Max Zeitinterv.
    
    % schwappbedingung und Korridor 
    [v_c, v_ceq] = completeValidation(base_Zeitintervall, base_Achswinkel,regression_1,simulation_data);
    
%%%%%%%%%%%%%%Übernemen von c und ceq, die durch CompleteValidation übergeben wurden%%%%%%%%%%%%%%
    if length(v_c) >= 1
        c(end+1:end+length(v_c)) = v_c;
    end

    if length(v_ceq) >= 1
        ceq(end+1:end+length(v_ceq)) = v_ceq;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Kontrollen an den einzelnen achswinkeln%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:size(base_Achswinkel_Mod,2) 
         [~,veolocity,acceleration,~,~,splinePunkt] = spline(base_Achswinkel_Mod(:,i),base_Zeitintervall_mod,false);
                 
%         debug_place(place)
         
         % BASEPOINTS BOUNDS 720 c's 
         
             for j=1:size(base_Achswinkel,1) %for every point on every axis
%                 c(end+1) = base_Achswinkel(j,i) - max_jointangle(i);%joint angle
%                 c(end+1) = min_jointangle(i) - base_Achswinkel(j,i);
                c(end+1) = veolocity(splinePunkt(j)) - max_velocity(i);%velocity
                c(end+1) = min_velocity(i) - veolocity(splinePunkt(j));
                c(end+1) = acceleration(splinePunkt(j)) - max_acceleration(i);%acceleration
                c(end+1) = min_acceleration(i) - acceleration(splinePunkt(j));   
             end
            
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Ausgabe auf der Konsole%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
ceq_string = join(string( find(ceq ~= 0)   ), ',');
    c_string = join(string( find(c > 0)   ), ',');   
    position_c_fail = find(c > 0);
    c_value_string = [];
    %c_value_string = join(string( find(c > 0)   ), ',');
    for z = 1: length(position_c_fail)
        c_value_string(end+1,1) = c(position_c_fail(z));
    end
    dfdf = strjoin(string(c_value_string),',');

    %ausgabe der anzah der c Bedingungen und die anzahl der verletzten
    if ismissing(c_string) == false
        %fprintf('c(%d, %d) [%s] \n',length(c),l), length(find(c-1e-6 > 0)  ,c_value_string);
        %fprintf('c(%d, %d) [%s] \n',length(c),l)ength(find(c-1e-6 > 0) ,c_string);
        fprintf('c(%d, %d) [%s] \n',length(c),length(find(c-1e-6 > 0)) ,strjoin(string(round(c_value_string,1)),', '));
    else
        fprintf('Keine Bedingung verletzt')
        fprintf(num2str(length(c)));
       
    end

    if ismissing(ceq_string) == false 
        fprintf('ceq(%d, %d) [%s] \n',length(ceq),length(find(ceq ~= 0)) ,ceq_string);       
    end  
end