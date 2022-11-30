function [optimized_translational_values_sumTime] = sumTimeAndMM(optimized_translational_values)
    [row, collumn] = size(optimized_translational_values);
    
    optimized_translational_values_sumTime = optimized_translational_values;
    % Umrechnen derZeitintrvalle in absolute Zeit
    for k = 1:row
        if k == 1
            startOffset(1) = optimized_translational_values_sumTime(k,2) * 1000 ;
            startOffset(2) = optimized_translational_values_sumTime(k,3) * 1000 ;
            startOffset(3) = optimized_translational_values_sumTime(k,4) * 1000 ;
            optimized_translational_values_sumTime(k,2:4) = optimized_translational_values_sumTime(k,2:4) * 1000 - startOffset;
        else
            eintragEins = optimized_translational_values_sumTime(k,2)*1000;
            eintragZwei = optimized_translational_values_sumTime(k,3)*1000;
            eintragDrei = optimized_translational_values_sumTime(k,4)*1000;
            optimized_translational_values_sumTime(k,1) = optimized_translational_values_sumTime(k,1)+optimized_translational_values_sumTime(k-1,1);
            
            optimized_translational_values_sumTime(k,2) = eintragEins - startOffset(1);
            optimized_translational_values_sumTime(k,3) = eintragZwei - startOffset(2);
            optimized_translational_values_sumTime(k,4) = eintragDrei - startOffset(3);
        end 
    end
end

