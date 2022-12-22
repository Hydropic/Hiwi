function [fig] = plotWGBRGraph(EMIFile)

%%PLOTTE Weg, Geschwindigkeit, Beschleuinigung und Ruck der sechs Achsen

%READ FILES
lineOfEmi = regexp(fileread(EMIFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
data = dlmread(EMIFile,'',[startLine 0 endLine 6]);

%SET LIMITS for Velocity, Acceleration....
max_velocity = [90,85,80,80,80,100];
min_velocity = [-90,-85,-80,-80,-80,-100];
max_acceleration = [200,200,200,200,200,200];
min_acceleration = [-200,-200,-200,-200,-200,-200];
max_jerk = [2000, 2000, 2000, 2000, 2000,2000];
min_jerk = [-2000,-2000,-2000,-2000,-2000,-2000];

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

%For the limitlines
timeEnd = timeAxis{1,1};

%%Calculate Xlim and Ylim for all the graphs

%Path
maxPath = -10000;
minPath = 10000;
for i = 1:6
    [maxLimit] = max(path{i},[],2)
    [minLimit] = min(path{i},[],2)

    if maxLimit > maxPath
        maxPath = maxLimit
    end

    if minLimit < minPath
        minPath = minLimit
    end
end
YMaxLimPath = maxPath;
YMinLimPath = minPath;

%Velocity
maxVelocity = -10000;
minVelocity = 10000;
for i = 1:6
    [maxLimit] = max(velocityAxis{i},[],2)
    [minLimit] = min(velocityAxis{i},[],2)

    if maxLimit > maxVelocity
        maxVelocity = maxLimit
    end

    if minLimit < minVelocity
        minVelocity = minLimit
    end
end
YMaxLimVelocity = maxVelocity;
YMinLimVelocity = minVelocity;

%Acceleration
maxAcceleration = -10000;
minAcceleration = 10000;
for i = 1:6
    [maxLimit] = max(accelerationAxis{i},[],2)
    [minLimit] = min(accelerationAxis{i},[],2)

    if maxLimit > maxAcceleration
        maxAcceleration = maxLimit
    end

    if minLimit < minAcceleration
        minAcceleration = minLimit
    end
end
YMaxLimAcceleration = maxAcceleration;
YMinLimAcceleration = minAcceleration;

%Jerk
maxJerk = -10000;
minJerk = 10000;
for i = 1:6
    [maxLimit] = max(jerkAxis{i},[],2)
    [minLimit] = min(jerkAxis{i},[],2)

    if maxLimit > maxJerk
        maxJerk = maxLimit
    end

    if minLimit < minJerk
        minJerk = minLimit
    end
end
YMaxLimJerk = maxJerk;
YMinLimJerk = minJerk;

%Configure Figure and plot
fig = figure(1);


%%FIRST AXIS
subplot(4,6,1)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,1)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('1.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,7)
plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,1)),'Color','b')
grid on
hold on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');                   
hold off
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,13)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,1)),'Color','g')
grid on
hold on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');     
hold off
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,19)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,1)),'Color','k')
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
grid on
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

%%SECOND AXIS

subplot(4,6,2)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,2)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('2.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,8)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,2)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,14)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,2)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,20)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,2)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

%%THIRD AXIS

subplot(4,6,3)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,3)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('3.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,9)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,3)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,15)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,3)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,21)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,3)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

%%FOURTH AXIS

subplot(4,6,4)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,4)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('4.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,10)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,4)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,16)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,4)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,22)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,4)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

%%FIFTH AXIS

subplot(4,6,5)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,5)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('5.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,11)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,5)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,17)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,5)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,23)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,5)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

%%SIXTH AXIS

subplot(4,6,6)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,6)),'Color','r')
grid on
ylim([YMinLimPath YMaxLimPath])
xticks(0:1:timeEnd(end))
title('6.Achse', 'FontSize', 10)
ylabel('Strecke [mm]', 'FontSize', 10)

subplot(4,6,12)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,6)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimVelocity YMaxLimVelocity])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,18)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,6)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
ylim([YMinLimAcceleration YMaxLimAcceleration])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,24)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,6)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
xticks(0:1:timeEnd(end))
ylim([YMinLimJerk YMaxLimJerk])
ylabel('Ruck', 'FontSize', 10)

fig.WindowState = 'maximized';

saveas(fig,'WGBRGraph','svg')

end