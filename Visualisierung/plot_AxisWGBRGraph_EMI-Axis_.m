function [fig] = plotWGBRGraph(EMIFile)

%%PLOTTE Weg, Geschwindigkeit, Beschleuinigung und Ruck der sechs Achsen
EMIFile = "Data/Emily1_Axis.txt";

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
max_jerk = [2000];
min_jerk = [-2000];

%SET height and width of Resolution
height_of_Resolution = 1080; %Max is dependent on your screen size (For me it is 1080)
width_of_Resolution = 1920; %Max is dependent on your screen size (For me it is 1920)

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
maxDistance = 0;
for i = 1:6
    [maxLimit] = max(path{i},[],2)
    [minLimit] = min(path{i},[],2)
    
    if (maxLimit - minLimit) > maxDistance
        maxDistance = maxLimit - minLimit;
    end
    
end
%Calculating the distance of the X Axis for scaling
maxDistance = ceil(maxDistance/100)*100;
YMaxLimPaths = [];
YMinLimPaths = [];
for i = 1:6
    [maxLimit] = max(path{i},[],2)
    [minLimit] = min(path{i},[],2)
    Distance_to_be_added = (maxDistance - (maxLimit - minLimit))/2;
    YMaxLimPathAxis = maxLimit + Distance_to_be_added;
    YMinLimPathAxis = minLimit - Distance_to_be_added;
    YMaxLimPaths(end+1) = YMaxLimPathAxis;
    YMinLimPaths(end+1) = YMinLimPathAxis;
end

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

    [maxVelocity] = max(max_velocity,[],2)
    [minVelocity] = min(min_velocity,[],2)

    YMaxLimVelocity = maxVelocity+10;
    YMinLimVelocity = minVelocity-10;

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

    [maxAcceleration] = max(max_acceleration,[],2)
    [minAcceleration] = min(min_acceleration,[],2)

    YMaxLimAcceleration = maxAcceleration+20;
    YMinLimAcceleration = minAcceleration-20;

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

    [maxJerk] = max(max_jerk,[],2)
    [minJerk] = min(min_jerk,[],2)

    YMaxLimJerk = maxJerk+200;
    YMinLimJerk = minJerk-200;

%Configure Figure and plot
fig = figure(1);


%%FIRST AXIS
subplot(4,6,1)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,1)),'Color','r')
grid on
ylim([YMinLimPaths(1) YMaxLimPaths(1)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end)+1)
xtickangle(0)
title('Achse 1', 'FontSize', 10)
ylabel('Winkel [°]', 'FontSize', 10)

subplot(4,6,7)
plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,1)),'Color','b')
grid on
hold on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');                   
hold off
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', 10)

subplot(4,6,13)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,1)),'Color','g')
grid on
hold on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');     
hold off
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', 10)

subplot(4,6,19)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,1)),'Color','k')
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
grid on
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])
ylabel('Ruck [mm/s³]', 'FontSize', 10)
xlabel('Zeit [s]', 'FontSize', 10)

%%SECOND AXIS

subplot(4,6,2)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,2)),'Color','r')
grid on
ylim([YMinLimPaths(2) YMaxLimPaths(2)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
xtickangle(0)
title('Achse 2', 'FontSize', 10)

subplot(4,6,8)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,2)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])


subplot(4,6,14)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,2)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])


subplot(4,6,20)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,2)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])



%%THIRD AXIS

subplot(4,6,3)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,3)),'Color','r')
grid on
ylim([YMinLimPaths(3) YMaxLimPaths(3)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
xtickangle(0)
title('Achse 3', 'FontSize', 10)


subplot(4,6,9)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,3)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])


subplot(4,6,15)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,3)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])


subplot(4,6,21)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,3)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])



%%FOURTH AXIS

subplot(4,6,4)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,4)),'Color','r')
grid on
ylim([YMinLimPaths(4) YMaxLimPaths(4)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
xtickangle(0)
title('Achse 4', 'FontSize', 10)

subplot(4,6,10)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,4)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])


subplot(4,6,16)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,4)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])


subplot(4,6,22)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,4)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])



%%FIFTH AXIS

subplot(4,6,5)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,5)),'Color','r')
grid on
ylim([YMinLimPaths(5) YMaxLimPaths(5)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
xtickangle(0)
title('Achse 5', 'FontSize', 10)


subplot(4,6,11)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,5)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])


subplot(4,6,17)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,5)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])


subplot(4,6,23)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,5)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])



%%SIXTH AXIS

subplot(4,6,6)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,6)),'Color','r')
grid on
ylim([YMinLimPaths(6) YMaxLimPaths(6)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
xtickangle(0)
title('Achse 6', 'FontSize', 10)


subplot(4,6,12)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,6)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','black');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimVelocity YMaxLimVelocity])
xlim([0 timeEnd(end)])


subplot(4,6,18)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,6)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','black');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','black');
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimAcceleration YMaxLimAcceleration])
xlim([0 timeEnd(end)])


subplot(4,6,24)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,6)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','black');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','black'); 
xticks(0:1:timeEnd(end))
xtickangle(0)
ylim([YMinLimJerk YMaxLimJerk])
xlim([0 timeEnd(end)])



% fig.WindowState = 'maximized';
fig.Position = [0 0 width_of_Resolution height_of_Resolution];

saveas(fig,'WGBRGraph','jpg')

end