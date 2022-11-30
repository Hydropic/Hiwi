
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
length(wpts);[cos(-pi/2) 0  sin(-pi/2)   0;...
                 0          1   0           -0.5;...
                -sin(-pi/2) 0   cos(-pi/2)  -0.5225;...
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
