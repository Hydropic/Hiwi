function [optimization_values_matrix, conuterRow, counterColum] = transformValuseToMatrix(optimization_values)
   optimization_values_matrix = [];
   counterColum = 7;
   % GA: transform optimization_values from double zu array
   conuterRow = length(optimization_values)/counterColum;
   for w = 1:conuterRow:length(optimization_values)
       if isempty(optimization_values_matrix)
           for rowInTable = 0:conuterRow-1
               optimization_values_matrix(rowInTable+1,1) = optimization_values(w+rowInTable);
           end
       
       else
           for rowInTable = 0:conuterRow-1
               if rowInTable == 0
                   optimization_values_matrix(1,end+1) = optimization_values(w+rowInTable);
               else
                   optimization_values_matrix(rowInTable+1,end) = optimization_values(w+rowInTable);
               end                  
           end
       end
   end
end

