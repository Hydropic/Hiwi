function [optimized_translational_values_oriented, path_angular_deflection] = matchPendelPathData(path_angular_deflection,optimized_translational_values, xOrientationNull, yOrientationNull)

    % Zeitintervalle werden aufsummiert
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
            [val,idx]=min(abs(optimized_translational_values_oriented(x,1)-path_angular_deflection));     

            % Rotation um z der TCP ermitteln
            [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(optimized_translational_values_oriented(x,2:7));
            % Rotation auf x und y anwenden
            % eulerZYX = -10;
            angle_xx_yy = eulerZYX(1, 1);
            angle_yx_xy = eulerZYX(1, 1)+90;
            int_gt_0 = @(n) (rem(n,1) == 0) & (n > 0);

            positiv = int_gt_0(int64(angle_xx_yy));
            if angle_xx_yy < 0 && angle_xx_yy > -90 || angle_xx_yy < 360 && angle_xx_yy > 270
%                 if int_gt_0(int64(angle_xx_yy))
%                     angle_xx_yy = angle_xx_yy - 270
%                     angle_yx_xy = angle_yx_xy - 270
%                 else
%                     angle_xx_yy = angle_xx_yy
%                     angle_yx_xy = angle_yx_xy
%                 end
                timesMinusOne_xx = 1;
                timesMinusOne_yx = -1;
                timesMinusOne_yy = 1;
                timesMinusOne_xy = 1;
            elseif angle_xx_yy < -90 && angle_xx_yy > -180 || angle_xx_yy < 270 && angle_xx_yy > 180
%                 if int_gt_0(int64(angle_xx_yy))
%                     angle_xx_yy = angle_xx_yy - 180
%                     angle_yx_xy = angle_yx_xy - 180
%                 else
%                     angle_xx_yy = angle_xx_yy + 90
%                     angle_yx_xy = angle_yx_xy + 90
%                 end
                timesMinusOne_xx = -1;
                timesMinusOne_yx = -1;
                timesMinusOne_yy = -1;
                timesMinusOne_xy = 1;
            elseif angle_xx_yy < -180 && angle_xx_yy > -270 || angle_xx_yy < 180 && angle_xx_yy > 90
%                 if int_gt_0(int64(angle_xx_yy))
%                     angle_xx_yy = angle_xx_yy - 90
%                     angle_yx_xy = angle_yx_xy - 90
%                 else
%                     angle_xx_yy = angle_xx_yy + 180
%                     angle_yx_xy = angle_yx_xy + 180
%                 end
                timesMinusOne_xx = -1;
                timesMinusOne_yx = 1;
                timesMinusOne_yy = -1;
                timesMinusOne_xy = -1;                 
            elseif angle_xx_yy < -270 && angle_xx_yy > -360 || angle_xx_yy < 90 && angle_xx_yy > 0
%                 if int_gt_0(int64(angle_xx_yy))
%                     angle_xx_yy = angle_xx_yy
%                     angle_yx_xy = angle_yx_xy
%                 else
%                     angle_xx_yy = angle_xx_yy + 270
%                     angle_yx_xy = angle_yx_xy + 270
%                 end
                timesMinusOne_xx = 1;
                timesMinusOne_yx = 1;
                timesMinusOne_yy = 1;
                timesMinusOne_xy = -1; 
            else
                angle_xx_yy = []
                angle_yx_xy = []
            end
            e    =  cos(deg2rad(angle_xx_yy(1, 1)))
            ee   =  cos(deg2rad(angle_yx_xy(1, 1)))
            eee  =  cos(deg2rad(angle_xx_yy(1, 1)))
            eeee =  cos(deg2rad(angle_yx_xy(1, 1)))
            path_angular_deflection_TCP_xx = timesMinusOne_xx*cos(deg2rad(angle_xx_yy(1, 1))) * path_angular_deflection(idx(1), 2);
            path_angular_deflection_TCP_yx = timesMinusOne_yx*cos(deg2rad(angle_xx_yy(1, 1)-90)) * path_angular_deflection(idx(1), 3);

            path_angular_deflection_TCP_y = path_angular_deflection_TCP_xx + path_angular_deflection_TCP_yx;

            path_angular_deflection_TCP_yy = timesMinusOne_yy*cos(deg2rad(angle_xx_yy(1, 1))) * path_angular_deflection(idx(1), 3);
            path_angular_deflection_TCP_xy = timesMinusOne_xy*cos(deg2rad(angle_xx_yy(1, 1)-90)) * path_angular_deflection(idx(1), 2);
            
            path_angular_deflection_TCP_x = path_angular_deflection_TCP_yy + path_angular_deflection_TCP_xy;

            plotAcceleration(x,:) = [optimized_translational_values_oriented(x, 1), path_angular_deflection_TCP_x, path_angular_deflection_TCP_y]

            
            if yOrientationNull
                optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7)
            else
                % Achs 6 gleich der Auslenkung des Pendels in y
                optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7) + deg2rad(path_angular_deflection_TCP_y); % TODO: Fixen Wert entfernen      
            end

            if xOrientationNull
                optimized_translational_values_oriented(x, 6) = optimized_translational_values(x, 6)
            else
                % Achs 5 gleich der Auslenkung des Pendels in x
                optimized_translational_values_oriented(x, 6) = optimized_translational_values(x, 6) + deg2rad(path_angular_deflection_TCP_x); % TODO: Fixen Wert entfernen
            end

            % Achs 4 gleich dem gradienten von Anfang bis Endposition
            optimized_translational_values_oriented(x, 5) = optimized_translational_values(x, 5); % TODO: Fixen Wert entfernen
        end
       
%        figure;
%        plot(plotAcceleration(:,1), plotAcceleration(:,2), 'm')
%        hold on
%        plot(plotAcceleration(:,1), plotAcceleration(:,3), 'k')
%        hold on
  % Zeitintervalle werden von Summe auf Intervalle zurückgesetzt  
  optimized_translational_values_oriented(:,1) = optimized_translational_values(:,1);

  %% ================ Ausgleichskurve ==============================
  [timeSmoothSpline, smoothSpline_y, smoothSpline_x] = drawSpline(optimized_translational_values_oriented, plotAcceleration);

    for i= 1:length(smoothSpline_x)
        if yOrientationNull
         smoothSpline_y(:,i) = 0; % Hauptrotation um die 6. Achse
        end
        if xOrientationNull
         smoothSpline_x(:,i) = 0; 
        end 
    end    
    
    for  w = 1:3
        if length(smoothSpline_x) > length(smoothSpline_y)
            smoothSpline_y(end+1) = 0
        elseif length(smoothSpline_x) < length(smoothSpline_y)
            smoothSpline_x(end+1) = 0
        end
    end
    
    for x = 1:length(optimized_translational_values_oriented)
        [val_x,idx_x]=min(abs(optimized_translational_values_sumTime(x,1)-timeSmoothSpline));
        [val_y,idx_y]=min(abs(optimized_translational_values_sumTime(x,1)-timeSmoothSpline));

        if yOrientationNull
            optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7)
        else
            % Achs 6 gleich der Auslenkung des Pendels in y
            optimized_translational_values_oriented(x, 7) = optimized_translational_values(x, 7) + deg2rad(smoothSpline_y(idx_y)); % TODO: Fixen Wert entfernen      
        end
        
        if xOrientationNull
            optimized_translational_values_oriented(x, 6) = optimized_translational_values(x, 6)
        else
            % Achs 5 gleich der Auslenkung des Pendels in x
            optimized_translational_values_oriented(x, 6) = optimized_translational_values(x, 6) + deg2rad(smoothSpline_x(idx_x)); % TODO: Fixen Wert entfernen
        end
    end


    path_angular_deflection = [timeSmoothSpline', smoothSpline_y', smoothSpline_x'];
end

