function [fig] = plotSwapAnglesCompare(emiFile,emiFile2,XTitle,YTitle, Filename)

emiFile = "input/90Wasser10GlycerinNeueKelle.txt";
emiFile2 = "input/SiziliumtombakNeueKelle.txt";
XTitle = "Vergleich Schwappverhalten von Siliziumtombak und Wasser-Glycerin-Gemisch um die x-Achse";
YTitle = "Vergleich Schwappverhalten von Siliziumtombak und Wasser-Glycerin-Gemisch um die y-Achse";
Filename = "output/schwappvergleich";
lineTimeFile = 'input/BewegungsabschnittePunkten.txt';

%% PLOT ANGLES

%READ FILES
lineOfEmi = regexp(fileread(emiFile),'\n','split');
whichline = find(contains(lineOfEmi,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data = dlmread(emiFile,'',whichline);

lineOfEmi2 = regexp(fileread(emiFile2),'\n','split');
whichline2 = find(contains(lineOfEmi2,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data2 = dlmread(emiFile2,'',whichline2);

timeData = data(:,1)
timeData2 = data(:,1)

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

timeintervals2 = zeros(1,size(timeData2,1) - 1);
sizeofArray = length(timeData2) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals2(i) = timeData2(i+1) - timeData2(i);
end

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

lineTimeSecond = [];
if lineTimeFileBoolean
    dataXLSX = regexp(fileread(lineTimeFile),'\n','split')
    for ss = 1:length(dataXLSX)
        lineTimeSecond(ss) = str2double(cell2mat(dataXLSX(ss))); 
    end
    indicesOfClosestValuesSecond = findClosest(timeData2,lineTime);
end

[splineX, velocity, acceleration, ruck , time] =  splineOptimal(data(:,7),timeintervals,false);
[splineX2, velocity2, acceleration2, ruck2 , time2] =  splineOptimal(data2(:,7),timeintervals2,false);

%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
plot(time,splineX)
hold on
plot(time2, splineX2)
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', 'red', 'LineStyle', '--');
    end

    for a = 1:length(lineTimeSecond)
        xline([timeData2(indicesOfClosestValuesSecond(a)) timeData2(indicesOfClosestValuesSecond(a))],'Color', 'blue', 'LineStyle', '--');
    end
end
hold off
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
lgd = legend('Wasser-Glycerin-Gemisch', 'Siliziumtombak');
lgd.FontSize = 7;
title(XTitle)
xlabel 'Zeit [s]';
ylabel 'Schwapp-Winkel [°]'

[splineY, velocity, acceleration, ruck , time] =  splineOptimal(data(:,8),timeintervals,false);
[splineY2, velocity2, acceleration2, ruck2 , time2] =  splineOptimal(data2(:,8),timeintervals2,false);

subplot(2,1,2)
plot(time, splineY)
hold on
plot(time2,splineY2)
if lineTimeFileBoolean
    for a = 1:length(lineTime)
        xline([timeData(indicesOfClosestValues(a)) timeData(indicesOfClosestValues(a))],'Color', 'red', 'LineStyle', '--');
    end

    for a = 1:length(lineTimeSecond)
        xline([timeData2(indicesOfClosestValuesSecond(a)) timeData2(indicesOfClosestValuesSecond(a))],'Color', 'blue', 'LineStyle', '--');
    end
end
hold off
grid on
xlim([0 timeData(end)])
xticks(0:1:timeData(end)+1)
lgd2 = legend('Wasser-Glycerin-Gemisch', 'Siliziumtombak');
lgd2.FontSize = 7;
title(YTitle)
xlabel 'Zeit [s]';
ylabel 'Schwapp-Winkel [°]'

saveas(fig,Filename)
end
