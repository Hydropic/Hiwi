function [fig] = plotSwapAngles(emiFile, compressBolean)

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
%Compress angle values if necessary
if compressBolean == 0
    plot(time, spline)
else
    [valuesOfAngles] = compressAngleValues(spline, 10);
    plot(time, valuesOfAngles)
end
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
title('X Angle (Swap)')
xlabel 'Zeit [s]';
ylabel 'X Winkel [°]'

subplot(2,1,2)
[spline2, velocity, acceleration, ruck , time2] =  splineOptimal(data(:,8),timeintervals,false);
%Compress angle values if necessary
if compressBolean == 0
    plot(time, spline2)
else
    [valuesOfAngles] = compressAngleValues(spline2, 10);
    plot(time, valuesOfAngles)
end
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
title('Y Angle (Swap)')
xlabel 'Zeit [s]';
ylabel 'Y Winkel [°]'

saveas(fig,'SwapAngle.jpg' )
end
