function [tcppunkt, eulZYX, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen,RichtungGlobal)
    %Kopien der Matrizen der Roboterkonfiguration (eventuell mit globalen
    %Variablen oder Funktionsübergabe effizienter)
    tform = [cos(pi)   0 sin(pi)   0;
            0          1 0         0;
            -sin(pi)   0 cos(pi)   0.5225;
            0          0 0         1];
    tform2 =    [cos(-pi/2) 0  sin(-pi/2)   0;
                 0          1   0           -0.5;
                -sin(-pi/2) 0   cos(-pi/2)  -0.5225;
                 0          0   0           1];
    tform3 =    [cos(pi)  -sin(pi)   0      0;
                 sin(pi)   cos(pi)   0   -1.3;
                 0           0       1      0;
                 0           0       0      1];
    tform4 =    [1  0           0           0;
                 0  cos(pi/2)   -sin(pi/2)  0.712;
                 0  sin(pi/2)   cos(pi/2)   0;
                 0  0           0           1];
    tform5 =    [1  0           0               0;
                 0  cos(-pi/2)  -sin(-pi/2)     0;
                 0  sin(-pi/2)  cos(-pi/2)      -0.313;
                 0  0           0               1];
    tform6 =    [1      0           0                0;
                 0      cos(pi/2)   -sin(pi/2)       0.29;
                 0      sin(pi/2)   cos(pi/2)        0;
                 0      0           0                1];

    %Endeffektor:
    R7 = RotationUmZ(deg2rad(30))*RotationUmY(deg2rad(90))*RotationUmX(deg2rad(-90))*RotationUmZ(deg2rad(-45));
    
    V7 = [0 0 -0.450];
    tform7 = [R7    V7';
              0 0 0 1];

    %für jedes Gelenk die Rotationsmatrix berechen
    %1. die Rotationswerte des Koordinatensystems mit der aktuellen
    %Rotation des Roboters verrechnen 
    %2. die Rotationsmatrix mit der Verschiebung des Koordinatensystems zu
    %t zusammensetzten 
    rotiert1 = tform(1:3,1:3)*RotationUmZ(achsstellungen(1));
    rotiert1matrix = [rotiert1 tform(1:3,4);
                        0 0 0 1] ;
    
    rotiert2 = tform2(1:3,1:3)*RotationUmZ(achsstellungen(2));
    rotiert2matrix = [rotiert2 tform2(1:3,4);
                        0 0 0 1];
    
    rotiert3 = tform3(1:3,1:3)*RotationUmZ(achsstellungen(3));
    rotiert3matrix = [rotiert3 tform3(1:3,4);
                        0 0 0 1];
    
    rotiert4 = tform4(1:3,1:3)*RotationUmZ(achsstellungen(4));
    rotiert4matrix = [rotiert4 tform4(1:3,4);
                        0 0 0 1];
    
    rotiert5 = tform5(1:3,1:3)*RotationUmZ(achsstellungen(5));
    rotiert5matrix = [rotiert5 tform5(1:3,4);
                        0 0 0 1];
    
    rotiert6 = tform6(1:3,1:3)*RotationUmZ(achsstellungen(6));
    rotiert6matrix = [rotiert6 tform6(1:3,4);
                        0 0 0 1];
    
    %Berechnung der Lage des TCP im globalen Koordinatensystem 
    %basiert auf folgender Formel: %endpunkt = tform*(tform2*(tform3*(tform4*(tform5*(tform6*tform7(1:4,4))))));
    tcppunkt = rotiert1matrix*(rotiert2matrix*(rotiert3matrix*(rotiert4matrix*(rotiert5matrix*(rotiert6matrix*tform7(1:4,4))))));
    tcppunkt = tcppunkt(1:3);
    winkelmatrix = rotiert1matrix*(rotiert2matrix*(rotiert3matrix*(rotiert4matrix*(rotiert5matrix*(rotiert6matrix*tform7)))));
    
    %Zusatz um in Beschleunigung die Richtung im TCP Koordinatensystem zu
    %bestimmen; Standardwert noch relevant? Muss wohl drin bleiben wegen
    %dem Returnvalue d.h. Frage: besser als Nullvektor oder unnötiger
    %Rechenaufwand?
    %punkt = [0,1,0,1];
    %punkt_verschoben = winkelmatrix*punkt';
    %RichtungInTCP = punkt_verschoben(1:3)-tcppunkt;
    
    RichtungInTCP = [0,0,0,1];
    if exist('RichtungGlobal', 'var')
        RichtungInTCP = winkelmatrix*[RichtungGlobal;   
                                     0];
    end
   
    %validierung = tcppunkt+vektor;
    %hold on 
    %plot3(punkt_verschoben(1),punkt_verschoben(2),punkt_verschoben(3),"Marker","+")
    %plot3(validierung(1),validierung(2),validierung(3),"Marker", ".")
    winkelmatrixEUL = winkelmatrix(1:3,1:3);
    eulZYX = rotm2eul(winkelmatrixEUL, "ZYX");
    eulZYX = rad2deg(eulZYX);
end
