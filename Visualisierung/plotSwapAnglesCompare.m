function [fig] = plotSwapAnglesCompare(emiFile,emiFile2,XTitle,YTitle, Filename)

%% PLOT ANGLES

%READ FILES
lineOfEmi = regexp(fileread(emiFile),'\n','split');
whichline = find(contains(lineOfEmi,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data = dlmread(emiFile,'',whichline);

lineOfEmi2 = regexp(fileread(emiFile2),'\n','split');
whichline2 = find(contains(lineOfEmi2,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data2 = dlmread(emiFile2,'',whichline2);

% siziliumLine = regexp(fileread("SiziliumtombakNeueKelle.txt"),'\n','split');
% whichsiziliumLine = find(contains(siziliumLine,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
% siziliumData = dlmread("SiziliumtombakNeueKelle.txt",'',whichsiziliumLine);

timeData = data(:,1)
timeData2 = data(:,1)

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

timeintervals2 = zeros(1,size(timeData2,1) - 1);
sizeofArray = length(timeData2) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals2(i) = timeData2(i+1) - timeData2(i);
end

[spline, velocity, acceleration, ruck , time] =  splineOptimal(data(:,7),timeintervals,false);
[spline2, velocity2, acceleration2, ruck2 , time2] =  splineOptimal(data2(:,7),timeintervals2,false);
% [spline3, velocity3, acceleration3, ruck3 , time3] =  splineOptimal(siziliumData(:,7),timeintervals,false);

%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
plot(time,spline)

hold on
plot(time2, spline2)
% plot(time3, spline3)
hold off
legend('Kelle Ohne Wand', 'Kelle mit Wand')
title(XTitle)
xlabel 'Timestep';
ylabel 'XAngle'

[splineY, velocity, acceleration, ruck , time] =  splineOptimal(data(:,8),timeintervals,false);
[splineY2, velocity2, acceleration2, ruck2 , time2] =  splineOptimal(data2(:,8),timeintervals2,false);
% [splineY3, velocity3, acceleration3, ruck3 , time3] =  splineOptimal(siziliumData(:,8),timeintervals,false);

subplot(2,1,2)
plot(time, splineY)

hold on
plot(time2,splineY2)
% plot(time3, splineY3)
hold off
legend('Kelle Ohne Wand', 'Kelle mit Wand')
title(YTitle)
xlabel 'Timestep';
ylabel 'YAngle'

saveas(fig,Filename)
end
