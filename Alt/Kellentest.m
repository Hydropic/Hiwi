KSetUp;
numSamples = 20;
KPfadgenerator;

zeitintervale =  ones(size(minJerkPath,1)-1,1);
%minJerkPath = output.bestfeasible.x(:,2:7)
% [t1,td1,tdd1,tddd1,time1,place1] = spline(minJerkPath(:,1),zeitintervale,false);
% [t2,td2,tdd2,tddd2,time2,place2] = spline(minJerkPath(:,2),zeitintervale,false);
% [t3,td3,tdd3,tddd3,time3,place3] = spline(minJerkPath(:,3),zeitintervale,false);
% [t4,td4,tdd4,tddd4,time4,place4] = spline(minJerkPath(:,4),zeitintervale,false);
% [t5,td5,tdd5,tddd5,time5,place5] = spline(minJerkPath(:,5),zeitintervale,false);
% [t6,td6,tdd6,tddd6,time6,place6] = spline(minJerkPath(:,6),zeitintervale,false);
Bahnzeichnung(minJerkPath);
for j=1:size(minJerkPath,1)-2
    % nachfolger /nachnachfolger
    %n= [t1(place1(j)+1), t2(place1(j)+1), t3(place1(j)+1), t4(place1(j)+1), t5(place1(j)+1), t6(place1(j)+1)];
    %nn= [t1(place1(j)+2), t2(place1(j)+2), t3(place1(j)+2), t4(place1(j)+2), t5(place1(j)+2), t6(place1(j)+2)];
    [pos, kelle, WinkelmatrixTCP1] = vorwaertskinematik(minJerkPath(j,:)); 
    [posn, kellen, VektorInTCP, WinkelmatrixTCP] = vorwaertskinematik(minJerkPath(j+1,:)); 
    [posnn, kellenn] = vorwaertskinematik(minJerkPath(j+2,:)); 
    
    posTCP = WinkelmatrixTCP*[pos;
                                1];
    posnnTCP = WinkelmatrixTCP*[posnn; 
                                1];

    posTCP = posTCP(1:3);
    posnnTCP = posTCP(1:3);

    if j==1    
        fprintf("Punkt 1");
        fprintf("Werteglobal berechnet:")
        [acceleration, directionGlobal] = Beschleunigung(pos, pos, posn, (1),minJerkPath(j,:))
        hold on
        quiver3(pos(1),pos(2),pos(3),directionGlobal(1),directionGlobal(2),directionGlobal(3),'Black');
        %fprintf("Werte am TCP berechnet:")
        %[accelerationTCP, directionTCP] = Beschleunigung([0 0 0], [0 0 0], posnTCP, (1),minJerkPath(j,:))
        fprintf("#############################################################################\n");
    end
    % nachfolger /nachnachfolger
    if(j==12)
        test = "test";
        show(robot,minJerkPath(12+1,:));
    end
    fprintf("Punkt " + (j+1))
    [acceleration, directionGlobal,directionTCPAlternativ] = Beschleunigung(pos, posn, posnn, (0.1),minJerkPath((j+1),:));
    acceleration
    beschleunigungsrichtungXYZ = directionGlobal(1:3,:);
    directionTCPAlternativ
    hold on
    quiver3(posn(1),posn(2),posn(3),directionGlobal(1),directionGlobal(2),directionGlobal(3),'Black');
    fprintf("Werte am TCP berechnet:\n")
    %[accelerationTCP, directionTCP] = Beschleunigung(posTCP, [0 0 0]', posnnTCP, (1),minJerkPath(j,:))
    fprintf("#############################################################################\n");
    
    if j==size(minJerkPath,1)-2
        fprintf("Punkt " + (j+2))
        [acceleration, directionGlobal,directionTCP] = Beschleunigung(posn, posnn, posnn, (1),minJerkPath((j+2),:));
        acceleration
        beschleunigungsrichtungXYZ = directionGlobal(1:3,:)
        hold on
        quiver3(posnn(1),posnn(2),posnn(3),directionGlobal(1),directionGlobal(2),directionGlobal(3),'Black'); 
        fprintf("#############################################################################\n");
    end

    
end

%Bahnzeichnung(minJerkPath);
%laufbahn(robot,minJerkPath,1,false);