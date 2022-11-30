numSamples = 20;


rrt = manipulatorRRT(robot,{});
rrt.ValidationDistance = 0.2;

rng(0);

%Start- und Endpunktkonfiguration 
% !!!!!!!! Achse 6 Angepasst um in pos z richtung zu zeigen!!!!!!!!!!!!!!!!
startConfig = deg2rad([165.962315, -66.036604, 121.721424, -127.021183, 67.659026, 123.249821]);
goalConfig  = deg2rad([80.826709, -48.366271, 94.006036, -176.551890, 45.691660, 147.589939]);

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


%Pfadoptimierung
[q,qd,qdd,qddd,pp,tpts,tSamples] = minjerkpolytraj(path',initialGuess,numSamples);
minJerkPath = q';
