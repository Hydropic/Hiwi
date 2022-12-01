function [fig] = plotSwapAngles(emiFile)

%% PLOT ANGLES

%READ FILES
lineOfEmi = regexp(fileread(emiFile),'\n','split');
whichline = find(contains(lineOfEmi,'Timestep[s] | HighestPointXYZ[m] | AngleToY[°] | Distance[m] | AngleX[°] | AngleY[°]'));
data = dlmread(emiFile,'',whichline);

%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
plot(data(:,1), data(:,7))
title('X Angle (Swap)')
xlabel 'Timestep';
ylabel 'XAngle'

subplot(2,1,2)
plot(data(:,1), data(:,8))
title('Y Angle (Swap)')
xlabel 'Timestep';
ylabel 'YAngle'

saveas(fig,'SwapAngle.jpg' )
end
