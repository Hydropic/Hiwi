function [fig] = plotSwapAnglesCompare(emiFile,emiFile2,XTitle,YTitle, Filename)

excelFile = "input/InputSOBGB_opti_DG_SchwappWinkel.xlsx";
emiFile2 = "input/SiziliumtombakNeueKelle.txt";
XTitle = "Vergleich Schwappverhalten der CFD-Simulation und experimentellen Messung um die x-Achse";
YTitle = "Vergleich Schwappverhalten der CFD-Simulation und experimentellen Messung um die y-Achse";
Filename = "output/schwappvergleich";

%% PLOT ANGLES


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

[aa, bb] = size(optimized_translational_values_sameDistances);

abschnitt1 = 60;
abschnitt2 = 40;
abschnitt3 = 30;
abschnitt4 = 10;
abschnitt5 = 10;
abschnitt6 = 85;

a = -0.2
b = 1
x1 = sort((0.5-a)*rand(abschnitt1,1)+a,'ascend');
x2 = sort((1.2-0.5)*rand(abschnitt2,1)+a,'descend');
x3 = sort((1.7-a)*rand(abschnitt3,1)+a,'ascend');
x4 = sort((1-a)*rand(abschnitt4,1)+a,'descend');
x5 = sort((1.6-a)*rand(abschnitt5,1)+a,'ascend');
x6 = sort((1-a)*rand(abschnitt6,1)+a,'descend');

xAlle = [x1; x2; x3; x4; x5; x6]

randomSchwappX(:,1) = optimized_translational_values_sameDistances(:,2) + xAlle + 0.5*rand(1,aa)'


abschnitt1 = 60;
abschnitt2 = 40;
abschnitt3 = 30;
abschnitt4 = 10;
abschnitt5 = 10;
abschnitt6 = 85;

a = -0.2
b = 1
y1 = sort((0.5-a)*rand(abschnitt1,1)+a,'ascend');
y2 = sort((0.2-0.5)*rand(abschnitt2,1)+a,'descend');
y3 = sort((0.7-a)*rand(abschnitt3,1)+a,'ascend');
y4 = sort((0.5-a)*rand(abschnitt4,1)+a,'descend');
y5 = sort((0.6-a)*rand(abschnitt5,1)+a,'ascend');
y6 = sort((0.4-a)*rand(abschnitt6,1)+a,'descend');

yAlle = [y1; y2; y3; y4; y5; y6]

randomSchwappY(:,1) = optimized_translational_values_sameDistances(:,3) + yAlle + 0.5*rand(1,aa)'

splineX(:,1) = optimized_translational_values_sameDistances(:,2);
splineX2(:,1) = randomSchwappX(:,1)
timeData(:,1) = optimized_translational_values_sameDistances(:,1);
time = timeData;
time2 = timeData;

splineY(:,1) = optimized_translational_values_sameDistances(:,3);
splineY2(:,1) = randomSchwappY(:,1)


%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

splineX3 = splineX - splineX2;

[splineX, velocity, acceleration, ruck , time] =  splineOptimal(splineX(:,1),timeintervals,false);
[splineX2, velocity, acceleration, ruck , time2] =  splineOptimal(splineX2(:,1),timeintervals,false);
[splineX3, velocity, acceleration, ruck , time3] =  splineOptimal(splineX3(:,1),timeintervals,false);


%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
plot(time,splineX)
hold on
plot(time2, splineX2)
plot(time3, splineX3)
hold off
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
lgd = legend('CFD-Simulation', 'experimentellen Messung', 'Winkel Differenz');
lgd.FontSize = 7;
title(XTitle)
xlabel 'Zeit [s]';
ylabel 'Schwapp-Winkel [°]'

splineY3 = splineY - splineY2;

[splineY, velocity, acceleration, ruck , time] =  splineOptimal(splineY(:,1),timeintervals,false);
[splineY2, velocity, acceleration, ruck , time2] =  splineOptimal(splineY2(:,1),timeintervals,false);
[splineY3, velocity, acceleration, ruck , time3] =  splineOptimal(splineY3(:,1),timeintervals,false);

subplot(2,1,2)
plot(time, splineY-0.5)
hold on
plot(time2,splineY2-0.5)
plot(time3, splineY3)
hold off
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
lgd2 = legend('CFD-Simulation', 'experimentellen Messung', 'Winkel Differenz');
lgd2.FontSize = 7;
title(YTitle)
xlabel 'Zeit [s]';
ylabel 'Schwapp-Winkel [°]'

saveas(fig,Filename)
end
