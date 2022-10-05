function [timeSmoothSpline, smoothSpline_y, smoothSpline_x] = drawSpline(optimized_translational_values, path_angular_deflection)
show_spline(optimized_translational_values, 'w')
% metch the right x and y
path_angular_deflection_New(:, 1) = path_angular_deflection(:, 1);
path_angular_deflection_New(:, 2) = path_angular_deflection(:, 3);
path_angular_deflection_New(:, 3) = -path_angular_deflection(:, 2);


    %% ======================= Achse x =======================
%     time_y =   [0  0.32 0.85 1.25   1.5 1.95   path_angular_deflection_New(end, 1)];
%     winkel_y = [0 30  -0     -2  -24 -3      0];
    time_y =   [0  0.1   0.25 0.65   0.9  1.4  path_angular_deflection_New(end, 1)];
    winkel_y = [0  5    18   18      0  -25    0];

    splineDiscretization = 200;
    
    xx_y = linspace(0, time_y(1,end), splineDiscretization);
    yy_y = spline(time_y, winkel_y, xx_y);

    winkel_y_6 = [winkel_y', winkel_y', winkel_y', winkel_y', winkel_y', winkel_y'];

    for w=1:length(time_y)
        if w > 1
        timeInterval_y(1, w-1) = time_y(1, w) - time_y(1, w-1);
        else
        end
    end

    lin_xx_x = linspace(0, path_angular_deflection_New(end, 1), splineDiscretization);
    lin_yy_x = spline(path_angular_deflection_New(:, 1), path_angular_deflection_New(:, 2), lin_xx_x);
    delta_x = - yy_y + lin_yy_x
    [ymin,idx_min] = min(delta_x);
    [ymax,idx_max] = max(delta_x); 
    figure;
    plot(path_angular_deflection_New(:, 1),path_angular_deflection_New(:, 2), 'b')
    hold on;
    plot(time_y, winkel_y, 'bo');
    hold on;
    plot(xx_y, yy_y, 'b','LineWidth', 3);
    hold on;
    plot(xx_y, delta_x, 'g','LineWidth', 1);
    hold on;
    text(xx_y(idx_min),ymin,['ymin: ' num2str(ymin)]);
    text(xx_y(idx_max),ymax,['ymax: ' num2str(ymax)]);
    hold off;
    legend('rotation um y - Achs 6');

    %% ======================= Achse y =======================
%     time_x =   [0 0.15 0.65 1.15 1.33 1.45 1.85 path_angular_deflection_New(end, 1)];
%     winkel_x = [0 -3 -9   10    8   15   12  0];
%     time_x =   [0 0.20  0.4  1.0  1.4  1.6  path_angular_deflection_New(end, 1)];
%     winkel_x = [0 8     14  -18  25  25 0];
%     time_x =   [0 0.15  0.3   0.5  0.9  1.2 1.55  1.7  path_angular_deflection_New(end, 1)];
%     winkel_x = [0 0     -3   -3    3   10   6     3  0];
    time_x =   [0 0.20  0.35  0.9  1.2  1.55  path_angular_deflection_New(end, 1)];
    winkel_x = [0 0     0  0    0  0 0];
        
    xx_x = linspace(0, time_x(1,end), splineDiscretization);
    yy_x = spline(time_x, winkel_x, xx_x);

    figure;
    lin_xx_y = linspace(0, path_angular_deflection_New(end, 1), splineDiscretization);
    lin_yy_y = spline(path_angular_deflection_New(:, 1), path_angular_deflection_New(:, 3), lin_xx_y);
    delta_y = - yy_x + lin_yy_y
    [ymin,idx_min] = min(delta_y);
    [ymax,idx_max] = max(delta_y); 
    TFmax = islocalmax(delta_y);
    plot(xx_y, delta_y, 'g','LineWidth', 1);
    hold on;
    text(xx_y(idx_min),ymin,['ymin: ' num2str(ymin)]);
    text(xx_y(idx_max),ymax,['ymax: ' num2str(ymax)]);
        
    plot(path_angular_deflection_New(:, 1),path_angular_deflection_New(:, 3), 'r')
    hold on;
    plot(time_x, winkel_x, 'ro');
    hold on;
    plot(xx_x, yy_x, 'r','LineWidth', 3);
    hold off;
    legend('rotation um x - Achs 5');

    winkel_x_6 = [winkel_x', winkel_x', winkel_x', winkel_x', winkel_x', winkel_x']; 

    for w=1:length(time_x)
        if w > 1
        timeInterval_x(1, w-1) = time_x(1, w) - time_x(1, w-1);
        else
        end
    end

    [achsstellung_x,vilocity_x,acceleration_x,yerk_x,timeAbs_x,splinePunkt_x] = splineOptimal(winkel_x_6,timeInterval_x,false);
    [achsstellung_y,vilocity_y,acceleration_y,yerk_y,timeAbs_y,splinePunkt_y] = splineOptimal(winkel_y_6,timeInterval_y,false);

    
    timeAbs_x(1,end+1) = time_x(1,end);
    timeAbs_y(1,end+1) = time_y(1,end);
    achsstellung_x(1,end+1) = 0;
    achsstellung_y(1,end+1) = 0;

    timeSmoothSpline = timeAbs_x;
    smoothSpline_x = 1*achsstellung_x;
    smoothSpline_y = 1*achsstellung_y;
    accelerationShow = []
    accelerationShowInterval = [];

    accelerationShow(:,1) = timeAbs_x(2:100:end)
    for h = 1:length(accelerationShow)
        if h ==1
            accelerationShowInterval(h) = accelerationShow(h+1) - accelerationShow(h)
        else
            accelerationShowInterval(h) = accelerationShow(h) - accelerationShow(h-1)
        end  
    end
%     accelerationShow(:,1) = accelerationShowInterval
%     accelerationShow(:,2) = smoothSpline_x(2:100:end)
%     accelerationShow(:,3) = smoothSpline_x(2:100:end)
%     accelerationShow(:,4) = smoothSpline_x(2:100:end)
%     accelerationShow(:,5) = smoothSpline_y(2:100:end)
%     accelerationShow(:,6) = smoothSpline_y(2:100:end)
%     accelerationShow(:,7) = smoothSpline_y(2:100:end)

    %show_spline(accelerationShow, 'Bad Output');
%     figure;
%     plot(timeAbs_x, achsstellung_x); 
%     hold on;
%     plot(timeAbs_y, achsstellung_y); 
    
end

