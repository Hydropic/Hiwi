function Bahnzeichnung(pfad)
%Start und Endpunkt generieren
Startpunkt = vorwaertskinematik(pfad(1,:));
Endpunkt = vorwaertskinematik(pfad(size(pfad,1),:));

%Start und Endpunkt als Kreise anzeigen (wird später nochmal als . gezeichnet) 
hold on 
plot3(Startpunkt(1),Startpunkt(2),Startpunkt(3), "Marker", "o")

plot3(Endpunkt(1),Endpunkt(2),Endpunkt(3), "Marker", "o")
hold off

tcppunkte = zeros(size(pfad,1),3);
eulerZYX = zeros(size(pfad,1),3);

for i = 1:size(pfad,1)    
    [tcppunkte(i,:),eulerZYX(i,:)] = vorwaertskinematik(pfad(i,:));
    hold on
    plot3(tcppunkte(i,1),tcppunkte(i,2),tcppunkte(i,3), "Marker", ".")

    rot_Welt_zu_TCP = RotationUmZ(eulerZYX(i,1))*RotationUmY(eulerZYX(i,2))*RotationUmX(eulerZYX(i,3)); 
    Vorzugsrichtung_global = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
    Vorzugsrichtung = -rot_Welt_zu_TCP*Vorzugsrichtung_global; % die Transformation von Global in TCP-Koordinaten
    Vorzugsrichtung_proj = [Vorzugsrichtung(1),Vorzugsrichtung(2),0]*0.3/sqrt(Vorzugsrichtung(1)^2+Vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor
    
    plot3(tcppunkte(i,1),tcppunkte(i,2),tcppunkte(i,3),'-o','Color','b','MarkerSize',10,...
    'MarkerFaceColor','#D9FFFF')
    
    plot3(tcppunkte(i,1)+Vorzugsrichtung_proj(1), ...
    tcppunkte(i,2)+ Vorzugsrichtung_proj(2), ...
    tcppunkte(i,3)+Vorzugsrichtung_proj(3),'-o','Color','b','MarkerSize',10,...
    'MarkerFaceColor','#D9FFFF')

    plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung_proj(1)],...
    [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung_proj(2)], ...
    [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung_proj(3)],'Color','blue','LineWidth',1);

    %hold off
end



%Beschleunigungsvektoren einzeichnen:
m = load('MatrixKelleUngekipptStandard.mat')
for j=1:size(pfad,1)-2 
    [pos, kelle] = vorwaertskinematik(pfad(j,:)); 
    [posn, kellen] = vorwaertskinematik(pfad(j+1,:)); 
    [posnn, kellenn] = vorwaertskinematik(pfad(j+2,:)); 
    if j==1    
        fprintf("Punkt 1");
        [acceleration, direction, directionGlobal] = Beschleunigung(pos, pos, posn, (1),m.MatrixKelleUngekippt);
        %Variablen zum Printen im Command Window nochmals aufgeführt
        acceleration
        beschleunigungsrichtungXYZ = directionGlobal(1:3,:);
        hold on
        quiver3(pos(1),pos(2),pos(3),directionGlobal(1),directionGlobal(2),directionGlobal(3));
        %quiver3(pos(1),pos(2),pos(3),directionTCP(1),directionTCP(2),directionTCP(3));
        rx = kelle(3);
        ry = kelle(2); 
        rz = kelle(1); 
        fprintf("#############################################################################\n");
    end
    % nachfolger /nachnachfolger
    %fprintf("Punkt " + (j+1))
    [acceleration, direction, directionGlobal] = Beschleunigung(pos, posn, posnn, (1),m.MatrixKelleUngekippt);
    %Variablen zum Printen im Command Window nochmals aufgeführt
    acceleration;
    beschleunigungsrichtungXYZ = directionGlobal(1:3,:);
    hold on
    quiver3(posn(1),posn(2),posn(3),directionGlobal(1),directionGlobal(2),directionGlobal(3));
    rx = kellen(3);
    ry = kellen(2); 
    rz = kellen(1); 
    fprintf("#############################################################################\n");
end

%Um weitere Sachen anzugeben:
%[Nullstellung,NullstellungZYX] = vorwaertskinematik(robot.homeConfiguration);
%beschleunigung = Beschleunigung(tcppunkte(1,:),tcppunkte(2,:), tcppunkte(3,:), 0.13);
eulerZYX
%NullstellungZYX
end

