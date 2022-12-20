function [fig] = plotSwapAngles(emiFile)

%% PLOT ANGLES

%READ FILES
lineOfEmi = regexp(fileread(emiFile),'\n','split');
whichline = find(contains(lineOfEmi,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data = dlmread(emiFile,'',whichline);


timeData = data(:,1)

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
[spline, velocity, acceleration, ruck , time] =  splineOptimal(data(:,7),timeintervals,false);
plot(time, spline)
title('X Angle (Swap)')
xlabel 'Timestep';
ylabel 'XAngle'

subplot(2,1,2)
[spline2, velocity, acceleration, ruck , time2] =  splineOptimal(data(:,8),timeintervals,false);
plot(time2, spline2)
title('Y Angle (Swap)')
xlabel 'Timestep';
ylabel 'YAngle'

saveas(fig,'SwapAngle.jpg' )
end
