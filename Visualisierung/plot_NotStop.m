function [fig] = plot_3DPath(emiFile,timePointsFile)
%emiFile = 'input/InputSOBGB_opti_DG_Emily_TCP.txt';
emiFile = 'input/STOP_0_P1.EMI';
timePointsFile = 'input/BewegungsabschnittePunkten.txt';

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


X(2:2:end) = []; 
Y(2:2:end) = []; 
Z(2:2:end) = []; 
timeData(2:2:end) = []; 
X(2:2:end) = []; 
Y(2:2:end) = []; 
Z(2:2:end) = []; 
timeData(2:2:end) = []; 



%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

 X(3131:end) = [];
 timeintervals(3130:end) = [];

X(1:3090) = [];
timeintervals(1:3090) = [];

 lineTimeFileBoolean = false;


[splineX, velocityX, accelerationX, ruckX , timeX] =  splineOptimal(X,timeintervals,false);
[splineY, velocityY, accelerationY, ruckY , timeY] =  splineOptimal(Y,timeintervals,false);
[splineZ, velocityZ, accelerationZ, ruckZ , timeZ] =  splineOptimal(Z,timeintervals,false);
% 
% velocityX = 0.2*velocityX;
% accelerationX = 0.2*accelerationX;
% ruckX = 0.01*ruckX;

time = timeX;

% hold on
% plot(timeX,splineX)
% plot(timeX,velocityX)
% plot(timeX,accelerationX)
% plot(timeX,ruckX)
% lgd = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% hold off


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
hold on
plot(time, splineX)
hold off
grid on
% xticks(0:0.5:time(end))
% xlim([0 timeData(end)])
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([lineTime(1,a) lineTime(1,a)],'Color', colorTime, 'LineStyle', '--');
    end
end
title('Bewegungsprofil im globalen Arbeitsraum-KOS')
% lgd = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd.FontSize = 7;
xlabel 'Zeit [s]';
% ylim([-3100 2000])
% yticks(-3100:500:2000)
ylabel 'Strecke [mm]'
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')


subplot(3,2,2)
hold on
plot(time, velocityX)
hold off
grid on
xticks(0:1:time(end))
% xlim([0 timeData(end)])
title('Geschwindigkeit im globalen Arbeitsraum-KOS')
% lgd2 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd2.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit[mm/s]'
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')

subplot(3,2,3)
hold on
plot(time, accelerationX)
hold off
grid on
xticks(0:0.5:time(end))
% xlim([0 timeData(end)])
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([lineTime(1,a) lineTime(1,a)],'Color', colorTime, 'LineStyle', '--');
    end
end
% line([0 time(end)],[2.7 2.7],'Color', colorY, 'LineStyle', '--');
% line([0 time(end)],[-2.7 -2.7],'Color',colorY, 'LineStyle', '--');
title('Beschleuinigungprofil im globalen Arbeitsraum-KOS')
% lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'
% ylim([-5.5 5.5])
% yticks(-6:1:6)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')

subplot(3,2,4)
hold on
plot(time, ruckX)
hold off
grid on
xticks(0:0.5:time(end))
% xlim([0 timeData(end)])
title('Ruck im globalen Arbeitsraum-KOS')
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([lineTime(1,a) lineTime(1,a)],'Color', colorTime, 'LineStyle', '--');
    end
end
% lgd3 = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd3.FontSize = 7;
xlabel 'Zeit [s]';
ylabel 'Ruck [mm/s³]'
% ylim([-8.0 8.0])
% yticks(-8:2:8)
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
end