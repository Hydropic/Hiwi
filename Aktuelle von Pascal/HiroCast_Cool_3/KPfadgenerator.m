
rrt = manipulatorRRT(robot,{});
rrt.ValidationDistance = 0.2;

rng(0);

%Start- und Endpunktkonfiguration 
% !!!!!!!! Achse 6 Angepasst um in pos z richtung zu zeigen!!!!!!!!!!!!!!!!
startConfig = deg2rad([131.96 -45.96 87.87 89.46 -86.37 -47.94]);
goalConfig  = deg2rad([115.02 -60.61 125.26 71.82 -81.59 -24.3]);

path = plan(rrt,startConfig,goalConfig);

%Pfad erweitern und Wegpunkte herausschreiben
interpPath = interpolate(rrt,path);

%Startpunkt wieder mit Aufnehmen, weil interpolate komisch
interpPath(1,:) = startConfig;
wpts = interpPath';
length(wpts);[cos(-pi/2) 0  sin(-pi/2)   0;
                 0          1   0           -0.5;
                -sin(-pi/2) 0   cos(-pi/2)  -0.5225;
                 0          0   0           1];

%Zeiteinschätzung an der sich minjerkpolytraj orientiert
    %initialGuess = linspace(0,size(wpts,2)*0.2,size(wpts,2));
    initialGuess = [0,1,2];

    %Anzahl an Punkten die minjerkpolytraj ausspuckt
    %Auskommentiert um in OptimalControl mehrere Anläufe mit
    %unterschiedlich vielen Punkten zu starten
    numSamples = 20;

%Pfadoptimierung
[q,qd,qdd,qddd,pp,tpts,tSamples] = minjerkpolytraj(path',initialGuess,numSamples);
minJerkPath = q';

%{
minJerkPath = deg2rad([+0.000000  -2227.000000 -405.000000 +1615.000000 -90.000000 +0.000000 
  +0.048000  -2227.000000 -405.840476 +1615.000000 -90.000000 +0.250000
  +0.096000  -2227.000000 -406.680951 +1615.000000 -90.000000 +0.500000
  +0.144000  -2227.000000 -409.786161 +1615.000000 -90.000000 +1.500000
  +0.192000  -2227.000000 -415.234140 +1615.000000 -90.000000 +3.000000
  +0.240000  -2227.000000 -424.304134 +1615.000000 -90.000000 +4.000000
  +0.288000  -2227.000000 -435.359122 +1615.000000 -90.000000 +5.600000
  +0.336000  -2227.000000 -452.507750 +1615.000000 -90.000000 +6.750000
  +0.384000  -2227.000000 -475.904774 +1615.000000 -90.000000 +7.450000
  +0.432000  -2227.000000 -505.163008 +1615.000000 -90.000000 +7.600000
  +0.480000  -2227.000000 -540.236199 +1615.000000 -90.000000 +7.600000
  +0.528000  -2227.000000 -574.716897 +1615.000000 -90.000000 +7.600000
  +0.576000  -2227.000000 -615.579339 +1615.000000 -90.000000 +7.600000
  +0.624000  -2227.000000 -666.350561 +1615.000000 -90.000000 +7.600000
  +0.672000  -2227.000000 -722.831685 +1615.000000 -90.000000 +7.600000
  +0.720000  -2227.000000 -784.839188 +1615.000000 -90.000000 +7.600000
  +0.768000  -2227.000000 -851.563352 +1615.000000 -90.000000 +6.800000
  +0.816000  -2227.000000 -914.333039 +1615.000000 -90.000000 +5.400000
  +0.864000  -2227.000000 -974.633670 +1615.000000 -90.000000 +3.625000
  +0.912000  -2227.000000 -1045.056461 +1615.000000 -90.000000 -0.323243
  +0.960000  -2227.000000 -1113.104096 +1615.000000 -90.000000 -5.577960
  +1.008000  -2227.000000 -1177.168677 +1615.000000 -90.000000 -10.60779
  +1.056000  -2227.000000 -1235.956899 +1615.000000 -90.000000 -14.852754
  +1.104000  -2227.000000 -1286.313694 +1615.000000 -90.000000 -17.032130
  +1.152000  -2227.000000 -1324.805819 +1615.000000 -90.000000 -17.105815
  +1.200000  -2227.000000 -1366.988076 +1615.000000 -90.000000 -17.105815
  +1.248000  -2227.000000 -1403.237435 +1615.000000 -90.000000 -17.105815
  +1.296000  -2227.000000 -1433.553895 +1615.000000 -90.000000 -17.105815
  +1.344000  -2227.000000 -1457.937456 +1615.000000 -90.000000 -17.105815
  +1.392000  -2227.000000 -1476.404068 +1615.000000 -90.000000 -17.105815
  +1.440000  -2227.000000 -1489.218046 +1615.000000 -90.000000 -15.000000
  +1.488000  -2227.000000 -1495.830470 +1615.000000 -90.000000 -13.400000
  +1.536000  -2227.000000 -1501.041245 +1615.000000 -90.000000 -10.000000
  +1.584000  -2227.000000 -1503.820679 +1615.000000 -90.000000 -6.750000 
  +1.632000  -2227.000000 -1504.754968 +1615.000000 -90.000000 -4.500000 
  +1.680000  -2227.000000 -1504.759526 +1615.000000 -90.000000 -1.600000 ]);
%}

%Inverse Kinematik mit Mathe
%aik = analyticalInverseKinematics(robot);
%{
%generateIKFunction(aik,'robotIK');
eePosition = [-1.4475,   -0.9397,    2.2093];
eePose = trvec2tform(eePosition);
%showdetails(aik)

%aik.KinematicGroup

%hold on
%plot3(eePosition(1),eePosition(2),eePosition(3), "Marker", "o")
%plot3(-1.4475,   -0.9397,    2.2093, "Marker", "o")
%plotTransforms(eePosition,tform2quat(eePose))
%hold off

%ikConfig = robotIK(eePose)

%Inverse Kinematik mit Solver
ik = inverseKinematics('RigidBodyTree',robot);
[LoesungBitte,Info]= ik("endeffector",eePose, [0.00001 0.00001 0.00001 0.01 0.01 0.01],[deg2rad(60.82) deg2rad(-80.76) deg2rad(87.32) deg2rad(-74.21) deg2rad(-22.52) deg2rad(73.09)])
LosungSollteSein= [deg2rad(40.28) deg2rad(-108.79) deg2rad(112.42) deg2rad(-86.17) deg2rad(-42.28) deg2rad(84.88)]
%}
%Plots zum Testen:
%figure
%plot(tSamples,q)
%hold all
%plot(tpts,wpts,"x")
%plot(interpPath,"LineStyle","none","Marker","+")
%plot(tSamples, minJerkPath, "LineStyle","none","Marker","+")