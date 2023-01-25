function [fig] = plotSWGBGraphXLSX(emiFile,excelFile)
%% PLOT ANGLES
emiFile = 'input/InputSOBGB_opti_DG_Emily_TCP.txt';
excelFile = 'input/InputSOBGB_opti_DG_SchwappWinkel.xlsx';
AccFile = 'input/InputSOBGB_haendisch_DG_determinationAccelerationTCP.mat';
lineTimeFile = 'input/BewegungsabschnittePunkten.txt';


%READ XYZ
lineOfEmi = regexp(fileread(emiFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
dataKartesisch = dlmread(emiFile,'',[startLine 0 endLine 6]);

optimized_translational_values_sameDistances = {};

%READ EXCELFILE SWAP ANGLES
%dataXLSX = readcell('input/KomplexeOptiBahnMessung.xlsx');
% dataXLSX = readcell('input/InputSOBGB_haendisch_DG_SchwappWinkel.xlsx');
dataXLSX = readcell(excelFile); 
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

timeIntervals = [];
[www, rrr] = size(timeData);

timeInterval = timeData(end)/(www-1)

for fff = 1:www
    if fff == 1
    else
    timeIntervals(fff-1,:) = timeInterval;
    end
end


[ZRot, velocityX1, accelerationX1, ruckX1 , timeIntervalsRot] =  splineOptimal(ZRot,timeIntervals,false);
[YRot, velocityX1, accelerationX1, ruckX1 , timeIntervalsRot] =  splineOptimal(YRot,timeIntervals,false);

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

AccData = load(AccFile);
AccDatax2 = AccData.acceleration_xy_TCP(:,1)';
AccDatay2 = AccData.acceleration_xy_TCP(:,2)';
AccDataz2 = AccData.acceleration_xy_TCP(:,3)';
AccData_time(:) = AccData.zeit_kurz();
AccData_time(:) = AccData.zeit_kurz(2);
AccData_time(end) = [];

%%================================= EINFÜGEN ==============================

lineTimeFileBoolean = true;

%READ lineTimeFile
lineTime = [];
if lineTimeFileBoolean
    dataXLSX = regexp(fileread(lineTimeFile),'\n','split')
    for ss = 1:length(dataXLSX)
        lineTime(ss) = str2double(cell2mat(dataXLSX(ss))); 
    end
    indicesOfClosestValues = findClosest(timeData,lineTime);
end

[splineX1, velocityX1, accelerationX1, ruckX1 , time1] =  splineOptimal(AccDatax2,AccData_time,false);
[splineY1, velocityY1, accelerationY1, ruckY1 , timeY1] =  splineOptimal(AccDatay2,AccData_time,false);
[splineZ1, velocityZ1, accelerationZ1, ruckZ1 , timeZ1] =  splineOptimal(AccDataz2,AccData_time,false);

%%================================= EINFÜGEN ENDE =========================

%Calculate Velocity,Acceleration and Path
[splineX, velocityX2, accelerationX2, ruckX2 , time] =  splineOptimal(X,timeintervals,false);
[splineY, velocityY2, accelerationY2, ruckY2 , timeY] =  splineOptimal(Y,timeintervals,false);
[splineZ, velocityZ2, accelerationZ2, ruckZ2 , timeZ] =  splineOptimal(Z,timeintervals,false);
[spline, velocity, acceleration, ruck , timeSwap] =  splineOptimal(optimized_translational_values_sameDistances(:,2),timeintervalsSwap,false);
[spline2, velocity, acceleration, ruck , timeSwap2] =  splineOptimal(optimized_translational_values_sameDistances(:,3),timeintervalsSwap,false);

% Convert color code to 1-by-3 RGB array (0~1 each)
strx = '#0072BD';
colorX = sscanf(strx(2:end),'%2x%2x%2x',[1 3])/255;

stry = '#D95319';
colorY = sscanf(stry(2:end),'%2x%2x%2x',[1 3])/255;

strz = '#EDB120';
colorZ = sscanf(strz(2:end),'%2x%2x%2x',[1 3])/255;

strTime = '#3b3b3b';
colorTime = sscanf(strTime(2:end),'%2x%2x%2x',[1 3])/255;

%Configure Figure and plot
fig = figure(1);
fig.Position = [100 100 1000 900]; 

subplot(3,2,1)
plot(time, splineX)
hold on
plot(time, splineY)
plot(time, splineZ)
hold off
grid on
xticks(0:0.5:time(end))
xlim([0 timeData(end)])
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', colorTime, 'LineStyle', '--');
    end
end
title('Bewegungsprofil im globalen Arbeitsraum-KOS')
lgd = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd.FontSize = 7;
xlabel 'Zeit [s]';
ylim([-3100 2000])
yticks(-3100:500:2000)
ylabel 'Strecke [mm]'
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')


% subplot(3,2,2)
% plot(time, velocityX)
% hold on
% plot(time, velocityY)
% plot(time, velocityZ)
% hold off
% grid on
% xticks(0:1:time(end))
% xlim([0 timeData(end)])
% title('Geschwindigkeit')
% lgd2 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd2.FontSize = 7;
% xlabel 'Zeit [s]';
% ylabel 'Geschwindigkeit[mm/s]'

subplot(3,2,2)
plot(time1, splineX1)
hold on
plot(time1, splineY1)
plot(time1, splineZ1)
hold off
grid on
xticks(0:0.5:time1(end))
xlim([0 timeData(end)])
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', colorTime, 'LineStyle', '--');
    end
end
line([0 time1(end)],[2.7 2.7],'Color', colorY, 'LineStyle', '--');
line([0 time1(end)],[-2.7 -2.7],'Color',colorY, 'LineStyle', '--');
title('Beschleuinigungprofil im TCP-KOS')
lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'
ylim([-5.5 5.5])
yticks(-6:1:6)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')


subplot(3,2,3)
plot(timeIntervalsRot, YRot, 'Color', colorY)
hold on
plot(timeIntervalsRot, ZRot, 'Color', colorZ)
hold off
grid on
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', colorTime, 'LineStyle', '--');
    end
end
xticks(0:0.5:time(end))
xlim([0 timeData(end)])
title('Umorientierung der Kelle im TCP-KOS')
lgd3 = legend('Y-Rot','Z-Rot','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Winkel [°]'
ylim([-115 35])
yticks(-115:10:35)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')

subplot(3,2,4)
plot(time1, velocityX1)
hold on
plot(time1, velocityY1)
plot(time1, velocityZ1)
hold off
grid on
xticks(0:0.5:time1(end))
xlim([0 timeData(end)])
title('Ruck im TCP-KOS')
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', colorTime, 'LineStyle', '--');
    end
end
lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Ruck [mm/s³]'
ylim([-8.0 8.0])
yticks(-8:2:8)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')



% subplot(3,2,4)
% plot(time, ruckX)
% hold on
% plot(time, ruckY)
% plot(time, ruckZ)
% hold off
% grid on
% xticks(0:1:time(end))
% xlim([0 timeData(end)])
% title('Ruck')
% lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd3.FontSize = 7;
% xlabel 'Zeit [s]';
% ylabel 'Ruck[mm/s³]'


subplot(3,2,6)
plot(timeSwap, spline)
hold on
plot(timeSwap2, spline2)
hold off
grid on
xticks(0:0.5:time(end))
xlim([0 timeData(end)])
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([lineTime(1,a) lineTime(1,a)],'Color', colorTime, 'LineStyle', '--');
    end
end
title('Schwappwinkel im TCP-KOS')
line([0 time1(end)],[10 10],'Color', 'Black', 'LineStyle', '--');
line([0 time1(end)],[-10 -10],'Color','Black', 'LineStyle', '--');
lgd4 = legend('X-Rot','Y-Rot','Location','best');
lgd4.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Winkel [°]'
ylim([-12 12])
yticks(-12:2:12)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')



%saveas(fig,'SwapAngle.jpg' )
end