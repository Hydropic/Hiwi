function [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen,RichtungGlobal)
    %Kopien der Matrizen der Roboterkonfiguration (eventuell mit globalen
    %Variablen oder Funktionsübergabe effizienter)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 1: Base]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    phi1 = 0;
    tform = [cos(phi1)   0   sin(phi1)       0;
             0           1   0               0;
             -sin(phi1)  0   cos(phi1)  0.5225;
             0           0   0               1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 2: Schulter]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi2 = (pi/2)*3;
    tform2 =    [1   0           0               0.5;
                 0   cos(-phi2)  -sin(-phi2)       0;
                 0   sin(-phi2)  cos(-phi2)   0.5225;
             0   0           0                 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 3: Ellenbogen]%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi3 = 0;
    tform3 =    [1   0           0               1.3;
                 0   cos(-phi3)  -sin(-phi3)       0;
                 0   sin(-phi3)  cos(-phi3)        0;
                 0   0           0                 1];
%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 4: Rot Unterarm]%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi4 = pi/2;
    tform4_1 =   [cos(phi4)  0   sin(phi4)   0.712;
                 0           1   0             0;
                 -sin(phi4)  0   cos(phi4)     0;
                 0           0   0             1];
    
    tform4_2 =   [cos(pi/2)  -sin(pi/2)   0    0;
                 sin(pi/2)   cos(pi/2)    0    0;
                 0           0            1    0;
                 0           0            0    1];
    tform4 = tform4_1*tform4_2;
%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 5: Handgelenk]%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    phi5 = pi/2;
    tform5_1 =    [1  0           0                0;
                 0  cos(phi5)   -sin(phi5)       0;
                 0  sin(phi5)   cos(phi5)    0.313;
                 0  0           0                1];
    tform5_2 =   [cos(pi)  0   sin(pi)     0;
                 0         1   0           0;
                 -sin(pi)  0   cos(pi)     0;
                 0         0   0           1];
    
    tform5_3 =  [cos(pi/2)   -sin(pi/2)   0    0;
                 sin(pi/2)   cos(pi/2)    0    0;
                 0           0            1    0;
                 0           0            0    1];
     tform5 = tform5_1*tform5_2*tform5_3; 

% % % %      d = rotm2eul(tform5(1:3,1:3),'XYZ');
% % % %      display(d); zum testen
%%%%%%%%%%%%%%%%%%%%%%%%[Körper 6: Rot Handgelenk]%%%%%%%%%%%%%%%%%%%%%%%%%
    phi6 = -(pi/2)*3;
    tform6_1 =   [cos(phi6)  0   sin(phi6)    0.29;
                 0           1   0               0;
                 -sin(phi6)  0   cos(phi6)       0;
                 0           0   0               1];
    
    tform6_2 =  [cos(-pi/2)   -sin(-pi/2)   0    0;
                 sin(-pi/2)   cos(-pi/2)    0    0;
                 0           0            1    0;
                 0           0            0    1];
    tform6 = tform6_1*tform6_2;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 7: Kelle]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    R7 = RotationUmZ(deg2rad(90))*RotationUmY(deg2rad(-45))*RotationUmX(deg2rad(-90));%Intrinsisch
    R7_ =RotationUmZ(deg2rad(-90))*RotationUmY(deg2rad(-45))*RotationUmX(deg2rad(-90));%Extrinsisch
    V7 = [0 0 0.450];
    tform7 = [R7    V7';
              0 0 0 1];
    tform7_ = [R7_    V7';%test
              0 0 0 1];

    %für jedes Gelenk die Rotationsmatrix berechen
    %1. die Rotationswerte des Koordinatensystems mit der aktuellen
    %Rotation des Roboters verrechnen 
    %2. die Rotationsmatrix mit der Verschiebung des Koordinatensystems zu
    %t zusammensetzten 


    %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    %!!!!!!!!!Achtung achswinkel in vorwerzkinematig gegenläufig!!!!!!!!!!!
    %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    rotiert1 = tform(1:3,1:3)*RotationUmZ(-achsstellungen(1));
    rotiert1matrix = [rotiert1 tform(1:3,4);
                        0 0 0 1] ;
    
    
    rotiert2 = tform2(1:3,1:3)*RotationUmZ(-achsstellungen(2));
    rotiert2matrix = [rotiert2 tform2(1:3,4);
                        0 0 0 1];
    
    rotiert3 = tform3(1:3,1:3)*RotationUmZ(-achsstellungen(3));
    rotiert3matrix = [rotiert3 tform3(1:3,4);
                        0 0 0 1];
    
    rotiert4 = tform4(1:3,1:3)*RotationUmZ(-achsstellungen(4));
    rotiert4matrix = [rotiert4 tform4(1:3,4);
                        0 0 0 1];
    
    rotiert5 = tform5(1:3,1:3)*RotationUmZ(-achsstellungen(5));
    rotiert5matrix = [rotiert5 tform5(1:3,4);
                        0 0 0 1];
    
    rotiert6 = tform6(1:3,1:3)*RotationUmZ(-achsstellungen(6));
    rotiert6matrix = [rotiert6 tform6(1:3,4);
                        0 0 0 1];
    
    %Berechnung der Lage des TCP im globalen Koordinatensystem 
    %basiert auf folgender Formel: %endpunkt = tform*(tform2*(tform3*(tform4*(tform5*(tform6*tform7(1:4,4))))));
    tcppunkt = rotiert1matrix*(rotiert2matrix*(rotiert3matrix*(rotiert4matrix*(rotiert5matrix*(rotiert6matrix*tform7(1:4,4))))));
    tcppunkt_ = rotiert1matrix*(rotiert2matrix*(rotiert3matrix*(rotiert4matrix*(rotiert5matrix*(rotiert6matrix*tform7_(1:4,4))))));%test
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
        RichtungInTCP = winkelmatrix*[RichtungGlobal;0];   
                                     
    end
   
    %validierung = tcppunkt+vektor;
    %hold on 
    %plot3(punkt_verschoben(1),punkt_verschoben(2),punkt_verschoben(3),"Marker","+")
    %plot3(validierung(1),validierung(2),validierung(3),"Marker", ".")
    winkelmatrixEUL = winkelmatrix(1:3,1:3);
    eulZYX = rotm2eul(winkelmatrixEUL, "ZYX");
    eulZYX = rad2deg(eulZYX);

    eulXYZ = rotm2eul(winkelmatrixEUL,'XYZ');
    eulXYZ = rad2deg(eulXYZ);
    
% % % %     Vektor = [-31;30;58];
% % % %     Winkel = RotationUmZ(deg2rad(eulZYX(1,1)))*RotationUmY(deg2rad(eulZYX(1,2)))*RotationUmX(deg2rad(eulZYX(1,3)));
% % % %     Vektor1 = Winkel*Vektor;
% % % %     Winkel_Inv = inv(RotationUmZ(deg2rad(eulZYX(1,3))))*inv(RotationUmY(deg2rad(eulZYX(1,2))))*inv(RotationUmX(deg2rad(eulZYX(1,1))));
% % % %     Vektor2 = Winkel_Inv*Vektor1;
% % % %     display(Vektor2);
% % % %     a =5;
% % % %     display(Winkel);

end
