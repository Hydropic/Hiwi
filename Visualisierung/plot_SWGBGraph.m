function [fig] = plotSWGBGraph(emiFile)
%% PLOT ANGLES

%READ SWAP ANGLES
lineOfEmi = regexp(fileread(emiFile),'\n','split');
whichline = find(contains(lineOfEmi,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data = dlmread(emiFile,'',whichline);

%READ XYZ
lineOfEmi = regexp(fileread(emiFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
dataKartesisch = dlmread(emiFile,'',[startLine 0 endLine 6]);


%Put each axis in its own array
timeData = dataKartesisch(:,1);
X = dataKartesisch(:,2);
Y = dataKartesisch(:,3);
Z = dataKartesisch(:,4);

timeData = data(:,1)

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

%Calculate Velocity,Acceleration and Path
[splineX, velocityX, accelerationX, ruckX , timeX] =  splineOptimal(X,timeintervals,false);
[splineY, velocityY, accelerationY, ruckY , timeY] =  splineOptimal(Y,timeintervals,false);
[splineZ, velocityZ, accelerationZ, ruckZ , timeZ] =  splineOptimal(Z,timeintervals,false);
[spline, velocity, acceleration, ruck , time] =  splineOptimal(data(:,7),timeintervals,false);
[spline2, velocity, acceleration, ruck , time2] =  splineOptimal(data(:,8),timeintervals,false);

%Configure Figure and plot
fig = figure(1);
subplot(2,2,1)
plot(time, splineX)
hold on
plot(time, splineY)
plot(time, splineZ)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Strecke')
lgd = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Strecke[mm]'

subplot(2,2,2)
plot(time, velocityX)
hold on
plot(time, velocityY)
plot(time, velocityZ)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Geschwindigkeit')
lgd2 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd2.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit[mm/s]'

subplot(2,2,3)
plot(time, accelerationX)
hold on
plot(time, accelerationY)
plot(time, accelerationZ)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Beschleuinigung')
lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung[mm/s²]'

subplot(2,2,4)
plot(time, spline)
hold on
plot(time2, spline2)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Schwappwinkel')
lgd4 = legend('X-Rot','Y-Rot','Location','best');
lgd4.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Winkel [°]'

saveas(fig,'SwapAngle.jpg' )
end