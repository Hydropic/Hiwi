function [fig] = plot_3DPath(emiFile)
emiFile = 'input/InputSOBGB_opti_DG_Emily_TCP.txt';

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

%Calculate distance between timeintervals
timeintervals = zeros(1,size(timeData,1) - 1);
sizeofArray = length(timeData) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = timeData(i+1) - timeData(i);
end

%Calculate Velocity,Acceleration and Path
[splineX, velocityX, accelerationX, ruckX , timeX] =  splineOptimal(X,timeintervals,false);
[splineY, velocityY, accelerationY, ruckY , timeY] =  splineOptimal(Y,timeintervals,false);
[splineZ, velocityZ, accelerationZ, ruckZ , timeZ] =  splineOptimal(Z,timeintervals,false);

accelerationVectorsXYZ = [];
accelerationVectorsXYZ(:,1) = accelerationX;
accelerationVectorsXYZ(:,2) = accelerationY;
accelerationVectorsXYZ(:,3) = accelerationZ;
magnitudeAccelerationXYZ = vecnorm(accelerationVectorsXYZ,2,2);

accelerationVectorsXY = [];
accelerationVectorsXY(:,1) = accelerationX;
accelerationVectorsXY(:,2) = accelerationY;
magnitudeAccelerationXY = vecnorm(accelerationVectorsXY,2,2);

accelerationVectorsXZ = [];
accelerationVectorsXZ(:,1) = accelerationX;
accelerationVectorsXZ(:,2) = accelerationZ;
magnitudeAccelerationXZ = vecnorm(accelerationVectorsXZ,2,2);

accelerationVectorsYZ = [];
accelerationVectorsYZ(:,1) = accelerationY;
accelerationVectorsYZ(:,2) = accelerationZ;
magnitudeAccelerationYZ = vecnorm(accelerationVectorsYZ,2,2);

splineX(end + 1) = nan
splineY(end + 1) = nan
splineZ(end + 1) = nan
magnitudeAccelerationXYZ(end + 1) = nan
magnitudeAccelerationXY(end + 1) = nan
magnitudeAccelerationXZ(end + 1) = nan
magnitudeAccelerationYZ(end + 1) = nan

%Configure Figure and plot
fig = figure(1);
subplot(2,2,1)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Bewegungsprofil projeziert auf Z und Y')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(90,0)

subplot(2,2,2)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Bewegungsprofil projeziert auf Y und X')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(0,90)

subplot(2,2,3)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Bewegungsprofil projeziert auf Z und X')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(0,0)

subplot(2,2,4)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Bewegungsprofil im globalen Arbeitsraum-KOS')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(3)
% plot(time, splineX)
% hold on
% plot(time, splineY)
% plot(time, splineZ)
% hold off
% grid on
% xticks(0:1:time(end))
% xlim([0 timeData(end)])
% title('Strecke')
% lgd = legend('X-Kor','Y-Kor','Z-Kor','Location','best');
% lgd.FontSize = 7;
% xlabel 'Zeit [s]';
% ylabel 'Strecke[mm]'

end