function [fig] = plotSWGBGraphXLSX(emiFile,excelFile)
%% PLOT ANGLES

%READ XYZ
lineOfEmi = regexp(fileread(emiFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
dataKartesisch = dlmread(emiFile,'',[startLine 0 endLine 6]);

optimized_translational_values_sameDistances = {};

%READ EXCELFILE SWAP ANGLES
dataXLSX = readcell('input/KomplexeOptiBahnMessung.xlsx');
% dataXLSX = readcell(excelFile); 
lastIndex = 0;
for i = 1:length(dataXLSX(:,1))
    checkZero = dataXLSX(i,1);
    if(ismissing(checkZero{1}))
        lastIndex = i;
        break;
    end
    disp(dataXLSX(i,1));
    optimized_translational_values_sameDistances(end+1,1) = dataXLSX(i,1);
end
optimized_translational_values_sameDistances(:,2) = dataXLSX(1:lastIndex-1,2);
optimized_translational_values_sameDistances(:,3) = dataXLSX(1:lastIndex-1,3);
optimized_translational_values_sameDistances = cell2mat(optimized_translational_values_sameDistances);

%Put each axis in its own array
timeData = dataKartesisch(:,1);
X = dataKartesisch(:,2);
Y = dataKartesisch(:,3);
Z = dataKartesisch(:,4);
ZRot = dataKartesisch(:,5);
YRot = dataKartesisch(:,6);

% timeData = optimized_translational_values_sameDistances(:,1);

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

timeDataSwap = optimized_translational_values_sameDistances(:,1);
timeintervalsSwap = zeros(1,size(timeDataSwap,1) - 1);
sizeofArray = length(timeDataSwap) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervalsSwap(i) = timeDataSwap(i+1) - timeDataSwap(i);
end


%Calculate Velocity,Acceleration and Path
[splineX, velocityX, accelerationX, ruckX , time] =  splineOptimal(X,timeintervals,false);
[splineY, velocityY, accelerationY, ruckY , timeY] =  splineOptimal(Y,timeintervals,false);
[splineZ, velocityZ, accelerationZ, ruckZ , timeZ] =  splineOptimal(Z,timeintervals,false);
[spline, velocity, acceleration, ruck , timeSwap] =  splineOptimal(optimized_translational_values_sameDistances(:,2),timeintervalsSwap,false);
[spline2, velocity, acceleration, ruck , timeSwap2] =  splineOptimal(optimized_translational_values_sameDistances(:,3),timeintervalsSwap,false);

%Configure Figure and plot
fig = figure(1);
subplot(2,3,1)
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

subplot(2,3,2)
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

subplot(2,3,3)
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

subplot(2,3,4)
plot(time, ruckX)
hold on
plot(time, ruckY)
plot(time, ruckZ)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Ruck')
lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Ruck[mm/s³]'

subplot(2,3,5)
plot(timeData, YRot)
hold on
plot(timeData, ZRot)
hold off
grid on
xticks(0:1:time(end))
xlim([0 timeData(end)])
title('Umorientierung der Kelle')
lgd3 = legend('Y-Rot','Z-Rot','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Winkel [°]'

subplot(2,3,6)
plot(timeSwap, spline)
hold on
plot(timeSwap2, spline2)
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