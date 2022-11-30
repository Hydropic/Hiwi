function [optimized_time_Steps] = matchPendelTimeSteps(path_angular_deflection,optimized_translational_values)

    optimized_time_Steps = [];
    optimized_translational_values_sumTime = optimized_translational_values;
    for k = 1:length(optimized_translational_values)
        if k == 1
            optimized_translational_values_sumTime(k,1) = 0;
        else
            optimized_translational_values_sumTime(k,1) = optimized_translational_values_sumTime(k,1)+optimized_translational_values_sumTime(k-1,1);
        end 
    end
    
    optimized_translational_values_oriented = optimized_translational_values_sumTime;
    for i = 1:length(optimized_translational_values_oriented)
        for x = 1:length(optimized_translational_values_oriented)
            [val,idx]=min(abs(optimized_translational_values_oriented(x,1)-path_angular_deflection(:,1)));
            
            optimized_time_Steps(x, 1) = optimized_translational_values_sumTime(x, 1);
            optimized_time_Steps(x, 2) = path_angular_deflection(idx(1), 2);
            optimized_time_Steps(x, 3) = path_angular_deflection(idx(1), 3);

            path_angular_deflection(idx,1);

        end
    end    
end

