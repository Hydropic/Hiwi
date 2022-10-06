function [timeSmoothSpline, smoothSpline_y, smoothSpline_x] = drawSpline(optimized_translational_values, path_angular_deflection)
show_spline(optimized_translational_values, 'w')
% metch the right x and y
path_angular_deflection_New(:, 1) = path_angular_deflection(:, 1);
path_angular_deflection_New(:, 2) = path_angular_deflection(:, 3);
path_angular_deflection_New(:, 3) = -path_angular_deflection(:, 2);

% Glätten der notwendigen Umorientierung im Achse 6
x = path_angular_deflection_New(:, 1);
y = path_angular_deflection_New(:, 2);
p = 0.997;
sp = csaps(x,y,p);
smoothSpline = fnval(sp, x)
smoothSpline = smoothSpline - smoothSpline(1)
endOffset = smoothSpline(end);

for e = 1:length(smoothSpline)
    zaehler = length(smoothSpline)-1
    proz = (e-1)/zaehler;
    smoothSpline(e) = smoothSpline(e) - endOffset * proz^2
end

% figure
% hold on
% fnplt(sp, 'g');
% plot(x,y,'ko');
% plot(sp.breaks, smoothSpline, 'b')
% plot(x, y, 'r')
% hold off

    timestep = [];
    for t = 1: length(path_angular_deflection_New(:, 1))
        if t == length(path_angular_deflection_New(:, 1))
            % timestep(t, 1) = timestep(t-1, 1)
        else 
            timestep(t, 1) = path_angular_deflection_New(t+1, 1) - path_angular_deflection_New(t, 1)
        end
    end
    [achsstellung_x,vilocity_x,acceleration_x,yerk_x,timeAbs_x,splinePunkt_x] = splineOptimal(path_angular_deflection_New(:, 3),timestep,false);
    [achsstellung_y,vilocity_y,acceleration_y,yerk_y,timeAbs_y,splinePunkt_y] = splineOptimal(smoothSpline,timestep,false);
    
%     figure
%     hold on
%     plot(path_angular_deflection_New(:, 1), path_angular_deflection_New(:, 2))
%     hold on
%     plot(timeAbs_x, achsstellung_y);
%     hold off

    timeAbs_x(1,end+1) = path_angular_deflection_New(end, 1);
    timeAbs_y(1,end+1) = path_angular_deflection_New(end, 1);
    achsstellung_x(1,end+1) = 0;
    achsstellung_y(1,end+1) = 0;

    timeSmoothSpline = timeAbs_x;
    smoothSpline_x = 1*achsstellung_x;
    smoothSpline_y = 1*achsstellung_y;
    accelerationShow = []
    accelerationShowInterval = [];    
end

