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
    hold off
end

for j=1:size(pfad,1)-2 
    [pos, kelle] = vorwaertskinematik(pfad(j,:)); 
    [posn, kellen] = vorwaertskinematik(pfad(j+1,:)); 
    [posnn, kellenn] = vorwaertskinematik(pfad(j+2,:)); 

    if j==1    
        fprintf("Punkt 1");
        [acceleration, directionGlobal,directionTCP] = Beschleunigung(pos, pos, posn, (1),pfad(j,:));
        %Variablen zum Printen im Command Window nochmals aufgeführt
        acceleration
        beschleunigungsrichtungXYZ = directionGlobal(1:3,:)
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
    [acceleration, directionGlobal,directionTCP] = Beschleunigung(pos, posn, posnn, (1),pfad(j,:));
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

