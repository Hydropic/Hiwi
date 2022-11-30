function [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(optimized_translational_values, passes)

    angleTCP = [];
    position_TCP = [];
    [row_initial_values, collum_initial_values] = size(optimized_translational_values);

    if passes == 1
        % Achse 4, 5 und 6 auf eine liniare Bahn führen
        for t = 1:length(optimized_translational_values)
            optimized_translational_values(t,5) = optimized_translational_values(1,5)+((optimized_translational_values(end,5)-optimized_translational_values(1,5))/length(optimized_translational_values))*t;
            optimized_translational_values(t,6) = optimized_translational_values(1,6)+((optimized_translational_values(end,6)-optimized_translational_values(1,6))/length(optimized_translational_values))*t;
            optimized_translational_values(t,7) = optimized_translational_values(1,7)+((optimized_translational_values(end,7)-optimized_translational_values(1,7))/length(optimized_translational_values))*t;
        end
    end
    
    % Abziehen aller Intervalle bis auf Achse 4, 5 und 6
    optimization_values = optimized_translational_values;
    optimization_values(:,4) = [];
    optimization_values(:,3) = [];
    optimization_values(:,2) = [];
    optimization_values(:,1) = [];

    % Transformation der Achsen in TCP Koordinaten
    for e = 1:row_initial_values
        pose = [optimized_translational_values(e, 2), optimized_translational_values(e, 3), optimized_translational_values(e, 4), optimized_translational_values(e, 5), optimized_translational_values(e, 6), optimized_translational_values(e, 7)];
        % eulerZYX2 gibt die Orientierungen Rz, Ry, Rx zurück (in der Reihenfolge)
        [pos2, eulerZYX2,eulerXYZ2, y_direction2] = vorwaertskinematik(pose);  
        angleTCP(end+1,1) = optimized_translational_values(e, 1);
        angleTCP(end,2) = eulerZYX2(1);
        angleTCP(end,3) = eulerZYX2(2);
        angleTCP(end,4) = eulerZYX2(3);

        position_TCP(end+1,1) = optimized_translational_values(e, 1);
        position_TCP(end,2) = pos2(1);
        position_TCP(end,3) = pos2(2);
        position_TCP(end,4) = pos2(3);
    end    

    optimized_translational_values_clear = optimized_translational_values;

end

