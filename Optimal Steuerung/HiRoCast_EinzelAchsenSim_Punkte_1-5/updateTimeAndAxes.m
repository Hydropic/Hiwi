function [optimized_translational_values_update] = updateTimeAndAxes(optimization_values, optimized_translational_values)



    for p = 1: length(optimized_translational_values)
        optimized_translational_values(p, 5) = optimization_values(p, 1);
        optimized_translational_values(p, 6) = optimization_values(p, 2);
        optimized_translational_values(p, 7) = optimization_values(p, 3);
    end
    
    optimized_translational_values_update = optimized_translational_values;

    x = optimized_translational_values_update; 
    %save('SimResults.mat','x');

end

