function [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection,optimized_translational_values)

    optimized_translational_values_sumTime = optimized_translational_values;
    for k = 1:length(optimized_translational_values)
        if k == 1
            optimized_translational_values_sumTime(k,1) = 0;
        else
            optimized_translational_values_sumTime(k,1) = optimized_translational_values_sumTime(k,1)+optimized_translational_values_sumTime(k-1,1);
        end 
    end
    optimized_translational_values_oriented = optimized_translational_values_sumTime;

%     figure;
%     plot(path_angular_deflection);
    
    %% ==== Auslenkung des Pendels wird initial auf die Orientierung der Achsen 5 und 6 gelegt =======
    path_angular_deflection(:,3) = -1*path_angular_deflection(:,3);

        for x = 1:length(optimized_translational_values_oriented)
            % Nächsten Wert der Simulation des Pendelmodells mit der Bahn ermitteln
            [val,idx]=min(abs(optimized_translational_values_oriented(x,1)-path_angular_deflection(:,1)));                 

            % Achs 6 gleich der Auslenkung des Pendels in y
            optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7) + deg2rad(path_angular_deflection(idx,2));
        end

        % Start Offset
    nullXOffset = 5;
    for s = 1:10
        optimized_translational_values_oriented(s,7) = optimized_translational_values_oriented(s,7) + deg2rad(nullXOffset)
        optimized_translational_values_oriented(end-s+1,7) = optimized_translational_values_oriented(end-s+1,7) - deg2rad(nullXOffset)
        nullXOffset = nullXOffset - 1
    end    

  optimized_translational_values_oriented(:,1) = optimized_translational_values(:,1);

%   %% ================ Ausgleichskurve ==============================
%   [timeSmoothSpline, smoothSpline_y, smoothSpline_x] = drawSpline(optimized_translational_values_oriented, plotAcceleration);
%        
%     for  w = 1:3
%         if length(smoothSpline_x) > length(smoothSpline_y)
%             smoothSpline_y(end+1) = 0
%         elseif length(smoothSpline_x) < length(smoothSpline_y)
%             smoothSpline_x(end+1) = 0
%         end
%     end
% 
%     for x = 1:length(optimized_translational_values_oriented)
%         [val_x,idx_x]=min(abs(optimized_translational_values_sumTime(x,1)-timeSmoothSpline));
%         [val_y,idx_y]=min(abs(optimized_translational_values_sumTime(x,1)-timeSmoothSpline));
% 
%         % Achs 6 gleich der Auslenkung des Pendels in y
%         optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7) + deg2rad(smoothSpline_y(idx_y));
%     end
% 
%     figure
%     plot(timeSmoothSpline, deg2rad(smoothSpline_y), 'b') % zeigt geglättete nötige Auslenkung
%     hold on;
%     plot(optimized_translational_values_sumTime(:,1), optimized_translational_values(:,7), 'g') % zeigt A6 in 0 Stellung
%     hold on;
%     plot(optimized_translational_values_sumTime(:,1), optimized_translational_values_oriented(:, 7), 'r') % zeigt A6 Kurve --> 0 Stellung + geglättete nötige Auslenkung
%     hold on;
%     legend('nötige Auslenkung','A6 in 0 Stellung','A6 --> 0 + nötige Auslenkung') 
% 
%     path_angular_deflection = [timeSmoothSpline', smoothSpline_y', smoothSpline_x'];
end

