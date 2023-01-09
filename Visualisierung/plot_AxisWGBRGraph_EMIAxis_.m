function [fig] = plotWGBRGraph(EMIFile)

%%PLOTTE Weg, Geschwindigkeit, Beschleuinigung und Ruck der sechs Achsen
EMIFile = "C:\Users\Ayman\Desktop\Neuer Ordner (8)\Gemisch\Hiwi\Visualisierung\input\Alt\Emily_Complex_A_gekuerzt.EMI";

%READ FILES
lineOfEmi = regexp(fileread(EMIFile),'\n','split');
startLine = find(contains(lineOfEmi,'[RECORDS]'));
endLine = find(contains(lineOfEmi,'[END]'));

endLine = endLine - 2;
data = dlmread(EMIFile,'',[startLine 0 endLine 6]);

%Fontsizes for the text in the figure
FontsizeXTicklabels = 12;
FontsizeYTicklabels = 12;
FontsizeLabels = 14;

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
marginVelocity = 20;

[maxVelocity] = max(max_velocity,[],2);
[minVelocity] = min(min_velocity,[],2);

YMaxLimVelocityAxises = maxVelocity+marginVelocity;
YMinLimVelocityAxises = minVelocity-marginVelocity;
YMaxLimitsVelocity = [];
YMinLimitsVelocity = [];

for i = 1:6
    distance = YMaxLimVelocityAxises - max_velocity(i);
    distanceAdded = marginVelocity - distance;
    YMaxLimitsVelocity(end+1) = YMaxLimVelocityAxises + distanceAdded;
    YMinLimitsVelocity(end+1) = YMinLimVelocityAxises - distanceAdded;
end



%Acceleration

marginAcceleration = 20;

[maxAcceleration] = max(max_acceleration,[],2)
[minAcceleration] = min(min_acceleration,[],2)

YMaxLimAccelerationAxises = maxAcceleration+marginAcceleration;
YMinLimAccelerationAxises = minAcceleration-marginAcceleration;
YMaxLimitsAcceleration = [];
YMinLimitsAcceleration = [];

for i = 1:6
    distance = YMaxLimAccelerationAxises - max_acceleration(i);
    distanceAdded = marginAcceleration - distance;
    YMaxLimitsAcceleration(end+1) = YMaxLimAccelerationAxises + distanceAdded;
    YMinLimitsAcceleration(end+1) = YMinLimAccelerationAxises - distanceAdded;
end

%Jerk

marginJerk = 200;

[maxJerk] = max(max_jerk,[],2);
[minJerk] = min(min_jerk,[],2);

YMaxLimJerkAxises = maxJerk+marginJerk;
YMinLimJerkAxises = minJerk-marginJerk;
YMaxLimitsJerk = [];
YMinLimitsJerk = [];

for i = 1:6
    distance = YMaxLimJerkAxises - max_jerk(1);
    distanceAdded = marginJerk - distance;
    YMaxLimitsJerk(end+1) = YMaxLimJerkAxises + distanceAdded;
    YMinLimitsJerk(end+1) = YMinLimJerkAxises - distanceAdded;
end

%Configure Figure and plot
fig = figure(1);


%%FIRST AXIS
subplot(4,6,1)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,1)),'Color','r')
grid on
ylim([YMinLimPaths(1) YMaxLimPaths(1)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end)+1)
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 1', 'FontSize', FontsizeLabels)
ylabel('Winkel [°]', 'FontSize', FontsizeLabels)

subplot(4,6,7)
plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,1)),'Color','b')
grid on
hold on
line([0 timeEnd(end)],[max_velocity(1) max_velocity(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(1) min_velocity(1)],'Color','red','LineStyle','--');                   
hold off
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(1) YMaxLimitsVelocity(1)])
xlim([0 timeEnd(end)])
ylabel('Geschwindigkeit [mm/s]', 'FontSize', FontsizeLabels)

subplot(4,6,13)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,1)),'Color','g')
grid on
hold on
line([0 timeEnd(end)],[max_acceleration(1) max_acceleration(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(1) min_acceleration(1)],'Color','red','LineStyle','--');     
hold off
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(1) YMaxLimitsAcceleration(1)])
xlim([0 timeEnd(end)])
ylabel('Beschleuinigung [mm/s²]', 'FontSize', FontsizeLabels)

subplot(4,6,19)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,1)),'Color','k')
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--'); 
grid on
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(1) YMaxLimitsJerk(1)])
xlim([0 timeEnd(end)])
ylabel('Ruck [mm/s³]', 'FontSize', FontsizeLabels)
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)

%%SECOND AXIS

subplot(4,6,2)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,2)),'Color','r')
grid on
ylim([YMinLimPaths(2) YMaxLimPaths(2)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 2', 'FontSize', FontsizeLabels)

subplot(4,6,8)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,2)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(2) max_velocity(2)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(2) min_velocity(2)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(2) YMaxLimitsVelocity(2)])
xlim([0 timeEnd(end)])


subplot(4,6,14)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,2)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(2) max_acceleration(2)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(2) min_acceleration(2)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(2) YMaxLimitsAcceleration(2)])
xlim([0 timeEnd(end)])


subplot(4,6,20)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,2)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(2) YMaxLimitsJerk(2)])
xlim([0 timeEnd(end)])
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)



%%THIRD AXIS

subplot(4,6,3)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,3)),'Color','r')
grid on
ylim([YMinLimPaths(3) YMaxLimPaths(3)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 3', 'FontSize', FontsizeLabels)


subplot(4,6,9)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,3)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(3) max_velocity(3)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(3) min_velocity(3)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(3) YMaxLimitsVelocity(3)])
xlim([0 timeEnd(end)])


subplot(4,6,15)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,3)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(3) max_acceleration(3)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(3) min_acceleration(3)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(3) YMaxLimitsAcceleration(3)])
xlim([0 timeEnd(end)])


subplot(4,6,21)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,3)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--'); 
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(3) YMaxLimitsJerk(3)])
xlim([0 timeEnd(end)])
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)



%%FOURTH AXIS

subplot(4,6,4)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,4)),'Color','r')
grid on
ylim([YMinLimPaths(4) YMaxLimPaths(4)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 4', 'FontSize', FontsizeLabels)

subplot(4,6,10)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,4)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(4) max_velocity(4)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(4) min_velocity(4)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(4) YMaxLimitsVelocity(4)])
xlim([0 timeEnd(end)])


subplot(4,6,16)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,4)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(4) max_acceleration(4)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(4) min_acceleration(4)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(4) YMaxLimitsAcceleration(4)])
xlim([0 timeEnd(end)])


subplot(4,6,22)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,4)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(4) YMaxLimitsJerk(4)])
xlim([0 timeEnd(end)])
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)



%%FIFTH AXIS

subplot(4,6,5)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,5)),'Color','r')
grid on
ylim([YMinLimPaths(5) YMaxLimPaths(5)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 5', 'FontSize', FontsizeLabels)


subplot(4,6,11)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,5)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(5) max_velocity(5)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(5) min_velocity(5)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(5) YMaxLimitsVelocity(5)])
xlim([0 timeEnd(end)])


subplot(4,6,17)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,5)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(5) max_acceleration(5)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(5) min_acceleration(5)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(5) YMaxLimitsAcceleration(5)])
xlim([0 timeEnd(end)])


subplot(4,6,23)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,5)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(5) YMaxLimitsJerk(5)])
xlim([0 timeEnd(end)])
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)



%%SIXTH AXIS

subplot(4,6,6)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,6)),'Color','r')
grid on
ylim([YMinLimPaths(6) YMaxLimPaths(6)])
xlim([0 timeEnd(end)])
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
title('Achse 6', 'FontSize', FontsizeLabels)


subplot(4,6,12)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,6)),'Color','b')
grid on
line([0 timeEnd(end)],[max_velocity(6) max_velocity(6)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_velocity(6) min_velocity(6)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsVelocity(6) YMaxLimitsVelocity(6)])
xlim([0 timeEnd(end)])


subplot(4,6,18)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,6)),'Color','g')
grid on
line([0 timeEnd(end)],[max_acceleration(6) max_acceleration(6)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_acceleration(6) min_acceleration(6)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsAcceleration(6) YMaxLimitsAcceleration(6)])
xlim([0 timeEnd(end)])


subplot(4,6,24)
plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,6)),'Color','k')
grid on
line([0 timeEnd(end)],[max_jerk(1) max_jerk(1)],'Color','red','LineStyle','--');
line([0 timeEnd(end)],[min_jerk(1) min_jerk(1)],'Color','red','LineStyle','--');
xticks(0:1:timeEnd(end))
ax = gca;
labels = string(ax.XAxis.TickLabels);
labels(2:2:end) = nan;
ax.XAxis.TickLabels = labels;
ax.XAxis.FontSize = FontsizeXTicklabels;
ax.YAxis.FontSize = FontsizeYTicklabels;
xtickangle(0)
ylim([YMinLimitsJerk(6) YMaxLimitsJerk(6)])
xlim([0 timeEnd(end)])
xlabel('Zeit [s]', 'FontSize', FontsizeLabels)



% fig.WindowState = 'maximized';
fig.Position = [0 0 width_of_Resolution height_of_Resolution];

saveas(fig,'WGBRGraph','jpg')

end