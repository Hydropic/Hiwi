function [fig] = plot_3DPath(emiFile,timePointsFile)
emiFile = "C:\Users\Ayman\Desktop\Neuer Ordner (8)\Gemisch\Hiwi\Visualisierung\input\InputSOBGB_opti_DG_Emily_TCP_New.txt";
timePointsFile = "C:\Users\Ayman\Desktop\Neuer Ordner (8)\Gemisch\Hiwi\Visualisierung\input\BewegungsabschnittePunkten.txt";

%SET VIEWS
viewProjektionZY = [-90, 0];
viewProjektionYX = [0, 90];
viewProjektionZX = [0, 0];
view3D = [-37.5, 30];

%SET LIMITS
XAxisLimit = [-2000, 2000];
YAxisLimit = [-3000, -1000];
ZAxisLimit = [500, 2000];


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

%READ TIMEPOINTS
fileID = fopen(timePointsFile,'r');
formatSpec = '%f';
timePoints = fscanf(fileID,formatSpec);
indexOfPoints = [];

for i = 1:length(timePoints)
    indexOfPoints(end + 1) = findClosest(timeData,timePoints(i));
end

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

splineX(end + 1) = nan;
splineY(end + 1) = nan;
splineZ(end + 1) = nan;
magnitudeAccelerationXYZ(end + 1) = nan;

%Plot Parameters
lineSize = 2;
greenToOrangeToRed = [linspace(0,1,128)' linspace(0.8,0.5,128)' linspace(0,0,128)';
                      linspace(1,1,128)' linspace(0.5,0,128)' linspace(0,0,128)'];
customColormap = colormap(gca,jet);
labels = [];
for i = 1:length(X(indexOfPoints))
labels(end + 1) = num2str(i) - '0';
end

%Set up robot
KSetUp;
% KPfadgenerator;
minJerkPath = [];
for i = 1:length(X(indexOfPoints))
    test  = dataKartesisch(indexOfPoints(i), 2:end);
    minJerkPath(end+1,:) = dataKartesisch(indexOfPoints(i), 2:end);
end
% laufbahn(robot,minJerkPath,1,true)
% show(robot,-minJerkPath(1,:),"Visuals","off", "Position",[5,5,5,0])
% show(robot,-minJerkPath(2,:),"Visuals","off", "Position",[8,8,8,0])
% show(robot,-minJerkPath(3,:),"Visuals","off", "Position",[9,9,9,0])


%Configure Figure and plot
fig = figure(1);
subplot(2,2,1)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
colormap(gca ,greenToOrangeToRed);
hold on
scatter3(X(indexOfPoints),Y(indexOfPoints),Z(indexOfPoints),50);
for i = 1:length(X(indexOfPoints))
    stringNum = num2str(i) + ".";
    text(X(indexOfPoints(i)),Y(indexOfPoints(i)), Z(indexOfPoints(i)),stringNum,'FontSize', 15)
end
hold off
daspect([1, 1, 1])
xlabel('X-Profil [mm]')
ylabel('Y-Profil [mm]')
zlabel('Z-Profil [mm]')
xlim(XAxisLimit)
ylim(YAxisLimit)
zlim(ZAxisLimit)
title('Bewegungsprofil projeziert auf Z und Y')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(viewProjektionZY)

subplot(2,2,2)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
colormap(gca ,greenToOrangeToRed);
hold on
scatter3(X(indexOfPoints),Y(indexOfPoints),Z(indexOfPoints),50);
for i = 1:length(X(indexOfPoints))
    stringNum = num2str(i) + ".";
    text(X(indexOfPoints(i)),Y(indexOfPoints(i)), Z(indexOfPoints(i)),stringNum,'FontSize', 15)
end
hold off
daspect([1, 1, 1])
xlabel('X-Profil [mm]')
ylabel('Y-Profil [mm]')
zlabel('Z-Profil [mm]')
xlim(XAxisLimit)
ylim(YAxisLimit)
zlim(ZAxisLimit)
title('Bewegungsprofil projeziert auf Y und X')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(viewProjektionYX)

subplot(2,2,3)
patch(splineX,splineY,splineZ,magnitudeAccelerationXYZ,'FaceColor','none','EdgeColor','interp')
colormap(gca ,greenToOrangeToRed);
hold on
scatter3(X(indexOfPoints),Y(indexOfPoints),Z(indexOfPoints),50);
for i = 1:length(X(indexOfPoints))
    stringNum = num2str(i) + ".";
    text(X(indexOfPoints(i)),Y(indexOfPoints(i)), Z(indexOfPoints(i)),stringNum,'FontSize', 15)
end
hold off
daspect([1, 1, 1])
xlabel('X-Profil [mm]')
ylabel('Y-Profil [mm]')
zlabel('Z-Profil [mm]')
xlim(XAxisLimit)
ylim(YAxisLimit)
zlim(ZAxisLimit)
title('Bewegungsprofil projeziert auf Z und X')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Beschleuinigung [mm/s²]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(viewProjektionZX)

subplot(2,2,4);

% show(robot,-minJerkPath(1,:), "Position",[X(indexOfPoints(1)),Y(indexOfPoints(1)), Z(indexOfPoints(1)),0],'PreservePlot',true)
patch(splineX,splineY,splineZ,splineZ,'FaceColor','none','EdgeColor','interp')
colormap(gca,customColormap(64:192,:));
caxis([min(splineZ) max(splineZ)])
% show(robot)
hold on
scatter3(X(indexOfPoints),Y(indexOfPoints),Z(indexOfPoints),50);
for i = 1:length(X(indexOfPoints))
    stringNum = num2str(i) + ".";
    text(X(indexOfPoints(i)),Y(indexOfPoints(i)), Z(indexOfPoints(i)),stringNum,'FontSize', 15)
end
hold off
% laufbahn(robot,minJerkPath,1,true)
daspect([1, 1, 1])
xlabel('X-Profil [mm]')
ylabel('Y-Profil [mm]')
zlabel('Z-Profil [mm]')
xlim(XAxisLimit)
ylim(YAxisLimit)
zlim(ZAxisLimit)
title('Bewegungsprofil im globalen Arbeitsraum-KOS')
set(gca, 'XMinorGrid','on', 'YMinorGrid','on')
a = colorbar;
ylabel(a,'Z-Profil [mm]','FontSize',12,'Rotation',270);
a.Label.Position(1) = 5;
view(view3D)

%Set LineWidth for all patch objects
patch_handles = findobj(gcf,'Type','patch'); % find all patch objects in the current figure

for k = 1:length(patch_handles)
    set(patch_handles(k),'LineWidth',lineSize); %set the linewidth to 2 for all patch objects
end

% show(robot,-minJerkPath(1,:), "Position",[X(indexOfPoints(1)),Y(indexOfPoints(1)), Z(indexOfPoints(1)),0],'Parent',ax)
end