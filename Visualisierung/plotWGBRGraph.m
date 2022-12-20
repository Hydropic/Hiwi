function [fig] = plotWGBRGraph(EMIFile)

%%PLOTTE Weg, Geschwindigkeit, Beschleuinigung und Ruck der sechs Achsen

%READ FILES
lineOfEmi = regexp(fileread(EMIFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
data = dlmread(EMIFile,'',[startLine 0 endLine 6]);


%Put each axis in its own array
timeData = data(:,1);
first_Axis = data(:,2);
second_Axis = data(:,3);
third_Axis = data(:,4);
fourth_Axis = data(:,5);
fifth_Axis = data(:,6);
sixth_Axis = data(:,7);

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end
disp(timeintervals)

%Using splineOptimal to derive the changed graphs
timeAxis = {};
path = {};
velocityAxis = {};
accelerationAxis = {};
jerkAxis = {};

[spline, velocity, acceleration, ruck , time] =  splineOptimal(first_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;

[spline, velocity, acceleration, ruck , time] =  splineOptimal(second_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;

[spline, velocity, acceleration, ruck , time] =  splineOptimal(third_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;

[spline, velocity, acceleration, ruck , time] =  splineOptimal(fourth_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;

[spline, velocity, acceleration, ruck , time] =  splineOptimal(fifth_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;

[spline, velocity, acceleration, ruck , time] =  splineOptimal(sixth_Axis,timeintervals,false);
timeAxis{end+1} = time;
path{end+1} = spline;
velocityAxis{end+1} = velocity;
accelerationAxis{end+1} = acceleration;
jerkAxis{end+1} = ruck;



%Configure Figure and plot
fig = figure(1);

%%FIRST AXIS
subplot(4,6,1)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,1)),'Color','r')
grid on
title('Erste Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,7)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,1)),'Color','b')
grid on
hold on
% line([0 3],[1.39 1.39],'Color','black');
% line([0 3],[-1.39 -1.39],'Color','black');                     
hold off
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,13)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,1)),'Color','g')
grid on
hold on
% line([0 3],[3.5 3.5],'Color','black');
% line([0 3],[-3.5 -3.5],'Color','black');
hold off
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,19)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,1)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'

%%SECOND AXIS

subplot(4,6,2)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,2)),'Color','r')
grid on
title('Zweite Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,8)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,2)),'Color','b')
grid on
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,14)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,2)),'Color','g')
grid on
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,20)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,2)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'

%%THIRD AXIS

subplot(4,6,3)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,3)),'Color','r')
grid on
title('Dritte Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,9)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,3)),'Color','b')
grid on
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,15)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,3)),'Color','g')
grid on
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,21)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,3)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'

%%FOURTH AXIS

subplot(4,6,4)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,4)),'Color','r')
grid on
title('Vierte Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,10)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,4)),'Color','b')
grid on
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,16)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,4)),'Color','g')
grid on
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,22)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,4)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'

%%FIFTH AXIS

subplot(4,6,5)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,5)),'Color','r')
grid on
title('Fünfte Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,11)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,5)),'Color','b')
grid on
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,17)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,5)),'Color','g')
grid on
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,23)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,5)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'

%%SIXTH AXIS

subplot(4,6,6)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,6)),'Color','r')
grid on
title('Sechste Achse')
xlabel 'Zeit [s]';
ylabel 'Strecke [mm]'

subplot(4,6,12)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,6)),'Color','b')
grid on
xlabel 'Zeit [s]';
ylabel 'Geschwindigkeit [mm/s]'

subplot(4,6,18)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,6)),'Color','g')
grid on
xlabel 'Zeit [s]';
ylabel 'Beschleuinigung [mm/s²]'

subplot(4,6,24)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,6)),'Color','k')
grid on
xlabel 'Zeit [s]';
ylabel 'Ruck'


end