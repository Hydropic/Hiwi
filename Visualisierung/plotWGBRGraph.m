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

%Using splineOptimal to derivate the changed graphs
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
subplot(2,2,1)
plot(cell2mat(timeAxis(:,1)), cell2mat(path(:,1)))
hold on
plot(cell2mat(timeAxis(:,2)), cell2mat(path(:,2)))
plot(cell2mat(timeAxis(:,3)), cell2mat(path(:,3)))
plot(cell2mat(timeAxis(:,4)), cell2mat(path(:,4)))
plot(cell2mat(timeAxis(:,5)), cell2mat(path(:,5)))
plot(cell2mat(timeAxis(:,6)), cell2mat(path(:,6)))
hold off
legend( 'First Axis', 'Second Axis', 'Third Axis', 'Fourth Axis', 'Fifth Axis', 'Sixth Axis')
title('Weg')
xlabel 'Timestep';
ylabel 'Angle'

subplot(2,2,2)

plot(cell2mat(timeAxis(:,1)), cell2mat(velocityAxis(:,1)))
hold on
plot(cell2mat(timeAxis(:,2)), cell2mat(velocityAxis(:,2)))
plot(cell2mat(timeAxis(:,3)), cell2mat(velocityAxis(:,3)))
plot(cell2mat(timeAxis(:,4)), cell2mat(velocityAxis(:,4)))
plot(cell2mat(timeAxis(:,5)), cell2mat(velocityAxis(:,5)))
plot(cell2mat(timeAxis(:,6)), cell2mat(velocityAxis(:,6)))
hold off
legend( 'First Axis', 'Second Axis', 'Third Axis', 'Fourth Axis', 'Fifth Axis', 'Sixth Axis')
title('Velocity')
xlabel 'Timestep';
ylabel 'Geschwindigkeit'

subplot(2,2,3)

plot(cell2mat(timeAxis(:,1)), cell2mat(accelerationAxis(:,1)))
hold on
plot(cell2mat(timeAxis(:,2)), cell2mat(accelerationAxis(:,2)))
plot(cell2mat(timeAxis(:,3)), cell2mat(accelerationAxis(:,3)))
plot(cell2mat(timeAxis(:,4)), cell2mat(accelerationAxis(:,4)))
plot(cell2mat(timeAxis(:,5)), cell2mat(accelerationAxis(:,5)))
plot(cell2mat(timeAxis(:,6)), cell2mat(accelerationAxis(:,6)))
hold off
legend( 'First Axis', 'Second Axis', 'Third Axis', 'Fourth Axis', 'Fifth Axis', 'Sixth Axis')
title('Acceleration')
xlabel 'Timestep';
ylabel 'Beschleuinigung'

subplot(2,2,4)

plot(cell2mat(timeAxis(:,1)), cell2mat(jerkAxis(:,1)))
hold on
plot(cell2mat(timeAxis(:,2)), cell2mat(jerkAxis(:,2)))
plot(cell2mat(timeAxis(:,3)), cell2mat(jerkAxis(:,3)))
plot(cell2mat(timeAxis(:,4)), cell2mat(jerkAxis(:,4)))
plot(cell2mat(timeAxis(:,5)), cell2mat(jerkAxis(:,5)))
plot(cell2mat(timeAxis(:,6)), cell2mat(jerkAxis(:,6)))
hold off
legend( 'First Axis', 'Second Axis', 'Third Axis', 'Fourth Axis', 'Fifth Axis', 'Sixth Axis')
title('Acceleration')
xlabel 'Timestep';
ylabel 'Beschleuinigung'


end